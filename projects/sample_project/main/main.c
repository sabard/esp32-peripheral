/* Ethernet Basic Example

   Took parts of code from: 
   expressif/esp32/examples/ethernet/basic/main/ethernet_example_main.c 
   expressif/esp32/examples/protocols/sockets/udp_server/main/udp_server.c 

   from https://github.com/espressif

   This script programs esp32 to light up an LED based on signals coming from PC
 */


#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#define PORT CONFIG_EXAMPLE_PORT
#define EXAMPLE_STATIC_IP_ADDR        CONFIG_EXAMPLE_STATIC_IP_ADDR
#define EXAMPLE_STATIC_NETMASK_ADDR   CONFIG_EXAMPLE_STATIC_NETMASK_ADDR
#define JUICE_GPIO  CONFIG_GPIO_JUICE
#define LIGHT_ON_GPIO  CONFIG_GPIO_LIGHT_ON
#define LIGHT_OFF_GPIO  CONFIG_GPIO_LIGHT_OFF
#define JUICE_PRIORITY  (tskIDLE_PRIORITY+5)
#define WATCH_PRIORITY (tskIDLE_PRIORITY+4)
#define LIGHT_ON_PRIORITY  (tskIDLE_PRIORITY+5)
#define LIGHT_OFF_PRIORITY  (tskIDLE_PRIORITY+5)


static const char *TAG = "eth_example";
char input;
int number;
static uint8_t state_juice=0;
static uint8_t state_light_on=0;
static uint8_t state_light_off=0;
SemaphoreHandle_t xSemaphore_juice = NULL;
SemaphoreHandle_t xSemaphore_light_on = NULL;
SemaphoreHandle_t xSemaphore_light_off = NULL;

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}


static void example_set_static_ip(esp_netif_t *netif)
{
    if (esp_netif_dhcpc_stop(netif) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop dhcp client");
        return;
    }
    esp_netif_ip_info_t ip;
    memset(&ip, 0 , sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(EXAMPLE_STATIC_IP_ADDR);
    ip.netmask.addr = ipaddr_addr(EXAMPLE_STATIC_NETMASK_ADDR);
    if (esp_netif_set_ip_info(netif, &ip) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ip info");
        return;
    }
    ESP_LOGD(TAG, "Success to set static ip: %s, netmask: %s", EXAMPLE_STATIC_IP_ADDR, EXAMPLE_STATIC_NETMASK_ADDR);
}


/** setting PHY config and connections and installing ethernet driver and connecting to tcp/ip stack  */
static void init_wesp32_eth()
{
	// Initialize mac config to default
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_mdc_gpio_num = 16;
	emac_config.smi_mdio_gpio_num = 17;
	esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);

	// Initialize phy config to default
  	eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
 	phy_config.phy_addr = 0;
 	phy_config.reset_gpio_num = -1;
	esp_eth_phy_t *phy = esp_eth_phy_new_rtl8201(&phy_config);

	// Initialize TCP/IP network interface (should be called only once in application)
	ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
	ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create new default instance of esp-netif for Ethernet
	esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
	esp_netif_t* eth_netif = esp_netif_new(&netif_config);

	esp_eth_handle_t eth_handle = NULL;
	esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
	ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

	esp_eth_netif_glue_handle_t eth_driver_handle =
						esp_eth_new_netif_glue(eth_handle);
    /* attach Ethernet driver to TCP/IP stack */	
	ESP_ERROR_CHECK(esp_netif_attach(eth_netif, eth_driver_handle));
    
	// Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    //ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
	
	example_set_static_ip(eth_netif);
    /* start Ethernet driver state machine */
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

}

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[1500];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");


#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif
        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 60;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);


        while (1) {
			ESP_LOGI(TAG, "debug4");
            ESP_LOGI(TAG, "Waiting for data");
			ESP_LOGI(TAG, "debug5");
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
			ESP_LOGI(TAG, "debug1");
			// Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
				ESP_LOGI(TAG, "debug2");	
                // Get the sender's ip address as string
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }
				ESP_LOGI(TAG, "debug3");
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
				ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
                if (rx_buffer[0] == 'j') {  
					xSemaphoreGive( xSemaphore_juice );
                    ESP_LOGI(TAG, "debug6");
				}
				if (rx_buffer[0] == 'l') {   
					xSemaphoreGive( xSemaphore_light_on );
                    ESP_LOGI(TAG, "debug7");
				}
				if (rx_buffer[0] == 'o'){  
					xSemaphoreGive( xSemaphore_light_off);
                    ESP_LOGI(TAG, "debug8");
				}
   

                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}


void vTaskJuice( void * pvParameters )
{
    for ( ;; )
    {
        if ( xSemaphoreTake( xSemaphore_juice, portMAX_DELAY) == pdTRUE )
        {
            printf("JUICE SWITCH\n");
            gpio_set_level(JUICE_GPIO, state_juice);
            ESP_LOGI(TAG, "state juice: %d", state_juice);
            state_juice=!state_juice;

        }
    }

}

void vTaskLight_on( void * pvParameters )
{
    for ( ;; )
    {
        if ( xSemaphoreTake( xSemaphore_light_on, portMAX_DELAY) == pdTRUE )
        {
            printf("LIGHT ON SWITCH\n");
            gpio_set_level(LIGHT_ON_GPIO, state_light_on);
            ESP_LOGI(TAG, "state light on: %d", state_light_on);
            state_light_on=!state_light_on;

        }
    }

}


void vTaskLight_off( void * pvParameters )
{
    for ( ;; )
    {
        if ( xSemaphoreTake( xSemaphore_light_off, portMAX_DELAY) == pdTRUE )
        {
            printf("LIGHT OFF SWITCH\n");
            gpio_set_level(LIGHT_OFF_GPIO, state_light_off);
            ESP_LOGI(TAG, "state light off: %d", state_light_off);
			state_light_off=!state_light_off;

        }
    }

}


static void configure_led(void)
{
      gpio_reset_pin(JUICE_GPIO);
      gpio_set_direction(JUICE_GPIO, GPIO_MODE_OUTPUT);
      gpio_reset_pin(LIGHT_ON_GPIO);
      gpio_set_direction(LIGHT_ON_GPIO, GPIO_MODE_OUTPUT);
      gpio_reset_pin(LIGHT_OFF_GPIO);
      gpio_set_direction(LIGHT_OFF_GPIO, GPIO_MODE_OUTPUT);

}

void app_main(void)
{
	init_wesp32_eth();
#ifdef CONFIG_EXAMPLE_IPV4
	xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, WATCH_PRIORITY, NULL);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
	xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET6, WATCH_PRIORITY, NULL);
#endif

	configure_led();

    // creating semaphore
    xSemaphore_juice = xSemaphoreCreateBinary();
	xSemaphore_light_on = xSemaphoreCreateBinary();
	xSemaphore_light_off = xSemaphoreCreateBinary();


    if (xSemaphore_juice !=NULL)
    {
        // creating tasks and their handles  
		TaskHandle_t xTaskJuice = NULL;
		xTaskCreate(vTaskJuice, "JUICE", 4096, NULL , JUICE_PRIORITY , &xTaskJuice);

    }
    else
    {
        printf("semaphore did not create sucessfully\n");
    
	}
    if (xSemaphore_light_on !=NULL)
    {
        // creating tasks and their handles  
		TaskHandle_t xTaskLight_on = NULL;
		xTaskCreate(vTaskLight_on, "LIGHT_ON", 4096, NULL , LIGHT_ON_PRIORITY , &xTaskLight_on);
    }
    else
    {
        printf("semaphore did not create sucessfully\n");
    
	}
    if (xSemaphore_light_off !=NULL)
    {
        // creating tasks and their handles  
		TaskHandle_t xTaskLight_off = NULL;
		xTaskCreate(vTaskLight_off, "LIGHT_OFF", 4096, NULL , LIGHT_OFF_PRIORITY , &xTaskLight_off);
    }
    else
    {
        printf("semaphore did not create sucessfully\n");
    
	}


}
