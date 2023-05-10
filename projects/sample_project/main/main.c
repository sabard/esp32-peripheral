/* Ethernet Basic Example

   Took parts of code from: 
   expressif/esp-idf/examples/ethernet/basic/main/ethernet_example_main.c 
   expressif/esp-idf/examples/protocols/sockets/udp_server/main/udp_server.c 
   expressif/esp-idf/examples/protocols/sockets/udp_client/main/udp_client.c 

   from https://github.com/espressif

   This script programs wEsp32 to set up a UDP client and server to communicate with a host computer. 
   You can control when wEsp32 triggers a UDP client task to send a packet to host computer by pressing 'c' on the keyboad
   wESP32 is also simultaneously listening for any packets with UDP server so you can send packets to wESP32 from host computer. 

    This program also has specific tasks that does specific actions on particular GPIO pins based on what the UDP packet string message is. -- This will be discontinued. Instead, rather than the host computer sending UDP string messages, it will send out a dictionary with various configurations for GPIO actions specified in the dict itself (i.e. what GPIO pin, what action -- toggle on/off, force on, force off, pulse, duration etc)

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
#define STATIC_IP_ADDR        CONFIG_EXAMPLE_STATIC_IP_ADDR
#define STATIC_NETMASK_ADDR   CONFIG_EXAMPLE_STATIC_NETMASK_ADDR
#define JUICE_GPIO  CONFIG_GPIO_JUICE
#define LIGHT_ON_GPIO  CONFIG_GPIO_LIGHT_ON
#define LIGHT_OFF_GPIO  CONFIG_GPIO_LIGHT_OFF
#define WATCH_PRIORITY (tskIDLE_PRIORITY+4)
#define SEND_PRIORITY (tskIDLE_PRIORITY+5) 
#define TOGGLE_PRIORITY  (tskIDLE_PRIORITY+5)
#define FORCE_ON_PRIORITY  (tskIDLE_PRIORITY+5)
#define FORCE_OFF_PRIORITY  (tskIDLE_PRIORITY+5)
#define PULSE_PRIORITY  (tskIDLE_PRIORITY+5)
#define PULSE_LENGTH_MSEC CONFIG_PULSE_LENGTH_MSEC
#define TIMEOUT_SOCKET_SEC CONFIG_TIMEOUT_SOCKET_SEC
#define TIMEOUT_SOCKET_USEC CONFIG_TIMEOUT_SOCKET_USEC
#define HOST_IP_ADDR CONFIG_HOST_IP_ADDR // computer's ip address communicating with lico   

static const char *TAG = "eth_example";
char input;
int number;
static uint8_t state_juice=0;
static uint8_t state_light_on=0;
static uint8_t state_light_off=0;
SemaphoreHandle_t xSemaphore_toggle = NULL;
SemaphoreHandle_t xSemaphore_force_on = NULL;
SemaphoreHandle_t xSemaphore_force_off = NULL;
SemaphoreHandle_t xSemaphore_pulse = NULL;
static int pin_char;
const TickType_t xDelay = PULSE_LENGTH_MSEC / portTICK_PERIOD_MS;
SemaphoreHandle_t xSemaphore_send = NULL;
static const char *payload = "Message from ESP32 ";


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

// setting up the static ip address for wESP32 for host computer and wESP32 to be on the same subnet
static void example_set_static_ip(esp_netif_t *netif)
{
    if (esp_netif_dhcpc_stop(netif) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop dhcp client");
        return;
    }
    esp_netif_ip_info_t ip;
    memset(&ip, 0 , sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(STATIC_IP_ADDR);
    ip.netmask.addr = ipaddr_addr(STATIC_NETMASK_ADDR);
    if (esp_netif_set_ip_info(netif, &ip) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ip info");
        return;
    }
    ESP_LOGD(TAG, "Success to set static ip: %s, netmask: %s", STATIC_IP_ADDR, STATIC_NETMASK_ADDR);
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

    // Set a static ip address for wesp32
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
    struct sockaddr_in addr;
    int sock;

    while (1) {

        addr.sin_addr.s_addr =inet_addr(STATIC_IP_ADDR);  // htonl(INADDR_ANY);  
        addr.sin_family = AF_INET;
        addr.sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;

        sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Server socket created");


        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = TIMEOUT_SOCKET_SEC;
        timeout.tv_usec = TIMEOUT_SOCKET_USEC;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        int err = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
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
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);

                ESP_LOGI(TAG, "debug3");
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
				ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
				if (rx_buffer[0] == 't'){ // toggle
					pin_char = rx_buffer[1];
					xSemaphoreGive( xSemaphore_toggle );
				}
				if (rx_buffer[0] == 'f'){ // force on
					pin_char = rx_buffer[1];
					xSemaphoreGive( xSemaphore_force_on );
				}
				if (rx_buffer[0] == 'g'){ // force off
					pin_char = rx_buffer[1];
					xSemaphoreGive( xSemaphore_force_off );
				}
				if (rx_buffer[0] == 'p'){ // pulse
					pin_char = rx_buffer[1];
					xSemaphoreGive( xSemaphore_pulse );
				}
 

            }
        }
    }
 

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
   
    vTaskDelete(NULL);
}


void vTaskToggle( void * pvParameters )
{
    for ( ;; )
    {
        if ( xSemaphoreTake( xSemaphore_toggle, portMAX_DELAY) == pdTRUE )
        {
			if (pin_char == 'j'){

            	printf("TOGGLE JUICE\n");
            	state_juice=!state_juice; 
				gpio_set_level(JUICE_GPIO, state_juice);
            	ESP_LOGI(TAG, "state juice: %d", state_juice);
			}
			if (pin_char == 'l'){

            	printf("TOGGLE LIGHT ON\n");
            	state_light_on=!state_light_on;
				gpio_set_level(LIGHT_ON_GPIO, state_light_on);
            	ESP_LOGI(TAG, "state light on: %d", state_light_on);
			}
			if (pin_char == 'o'){

            	printf("TOGGLE LIGHT OFF\n");
            	state_light_off=!state_light_off;
				gpio_set_level(LIGHT_OFF_GPIO, state_light_off);
            	ESP_LOGI(TAG, "state light off: %d", state_light_off);
			}

        }
    }

}


void vTaskForceOn( void * pvParameters )
{
    for ( ;; )
    {
        if ( xSemaphoreTake( xSemaphore_force_on, portMAX_DELAY) == pdTRUE )
        {
			if (pin_char == 'j'){
            	printf("FORCE ON JUICE\n");
            	state_juice=1; 
				gpio_set_level(JUICE_GPIO, state_juice);
            	ESP_LOGI(TAG, "state juice: %d", state_juice);
			}
			if (pin_char == 'l'){
            	printf("FORCE ON LIGHT ON\n");
            	state_light_on=1;
				gpio_set_level(LIGHT_ON_GPIO, state_light_on);
            	ESP_LOGI(TAG, "state light on: %d", state_light_on);
			}
			if (pin_char == 'o'){
            	printf("FORCE ON LIGHT OFF\n");
            	state_light_off=1;
				gpio_set_level(LIGHT_OFF_GPIO, state_light_off);
            	ESP_LOGI(TAG, "state light off: %d", state_light_off);
			}

        }
    }

}


void vTaskForceOff( void * pvParameters )
{
    for ( ;; )
    {
        if ( xSemaphoreTake( xSemaphore_force_off, portMAX_DELAY) == pdTRUE )
        {
			if (pin_char == 'j'){
            	printf("FORCE OFF JUICE\n");
            	state_juice=0; 
				gpio_set_level(JUICE_GPIO, state_juice);
            	ESP_LOGI(TAG, "state juice: %d", state_juice);
			}
			if (pin_char == 'l'){
            	printf("FORCE OFF LIGHT ON\n");
            	state_light_on=0;
				gpio_set_level(LIGHT_ON_GPIO, state_light_on);
            	ESP_LOGI(TAG, "state light on: %d", state_light_on);
			}
			if (pin_char == 'o'){
            	printf("FORCE OFF LIGHT OFF\n");
            	state_light_off=0;
				gpio_set_level(LIGHT_OFF_GPIO, state_light_off);
            	ESP_LOGI(TAG, "state light off: %d", state_light_off);
			}

        }
    }

}


void vTaskPulse( void * pvParameters )
{
    for ( ;; )
    {
        if ( xSemaphoreTake( xSemaphore_pulse, portMAX_DELAY) == pdTRUE )
        {
			if (pin_char == 'j'){
            	printf("PULSE JUICE\n");
            	if (state_juice==0) {
					state_juice=1;
            		ESP_LOGI(TAG, "state juice: %d", state_juice);
					gpio_set_level(JUICE_GPIO, state_juice);
					vTaskDelay(xDelay);
					state_juice=0;
					gpio_set_level(JUICE_GPIO, state_juice);
            		ESP_LOGI(TAG, "state juice: %d", state_juice);
				} else {
						printf("Cannot pulse due to state not being at 0");
				}
			}
			if (pin_char == 'l'){
            	printf("PULSE LIGHT ON\n");
            	if (state_light_on==0) {
					state_light_on=1;
            		ESP_LOGI(TAG, "state light on: %d", state_light_on);
					gpio_set_level(LIGHT_ON_GPIO, state_light_on);
					vTaskDelay(xDelay);
					state_light_on=0;
					gpio_set_level(LIGHT_ON_GPIO, state_light_on);
            		ESP_LOGI(TAG, "state light on: %d", state_light_on);
				} else {
						printf("Cannot pulse due to state not being at 0");
				}
			}
			if (pin_char == 'o'){
            	printf("PULSE LIGHT OFF\n");
            	if (state_light_off==0) {
					state_light_off=1;
            		ESP_LOGI(TAG, "state light off: %d", state_light_off);
					gpio_set_level(LIGHT_OFF_GPIO, state_light_off);
					vTaskDelay(xDelay);
					state_light_off=0;
					gpio_set_level(LIGHT_OFF_GPIO, state_light_off);
            		ESP_LOGI(TAG, "state light off: %d", state_light_off);
				} else {
						printf("Cannot pulse due to state not being at 0");
				}
			}


        }
    }

}


void vTaskWatch( void *pvParameters )
{
	for ( ;; )
	{
		input = getchar();
		if (input == 'c')
		{
			xSemaphoreGive( xSemaphore_send );
		}
		vTaskDelay( 100 / portTICK_PERIOD_MS );
	}

}

// UDP client task function
void udp_client_task( void * pvParameters )
{
    printf("debug19");
    int addr_family = 0;
    int ip_protocol = 0;
    
    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Client Socket created");
    
        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = TIMEOUT_SOCKET_SEC;
        timeout.tv_usec = TIMEOUT_SOCKET_USEC;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
        
    
        printf("debug20");
         
    	for ( ;; )
    	{
    		if ( xSemaphoreTake( xSemaphore_send, portMAX_DELAY) == pdTRUE )
    		{
                printf("debug21");
                int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                printf("debug22");
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
                printf("debug23");
                ESP_LOGI(TAG, "Message sent");	
    		}
    		printf("end of loop -- send\n");
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
	
    xTaskCreate(udp_server_task, "UDP_SERVER", 4096, (void*)AF_INET, WATCH_PRIORITY, NULL);

	configure_led();

    // creating semaphore
	xSemaphore_toggle = xSemaphoreCreateBinary();
	xSemaphore_force_on = xSemaphoreCreateBinary();
	xSemaphore_force_off = xSemaphoreCreateBinary();
	xSemaphore_pulse = xSemaphoreCreateBinary();
	xSemaphore_send = xSemaphoreCreateBinary();
    

	if (xSemaphore_toggle !=NULL)
    {
        // creating tasks and their handles  
		TaskHandle_t xTaskToggle = NULL;
		xTaskCreate(vTaskToggle, "TOGGLE", 4096, NULL , TOGGLE_PRIORITY , &xTaskToggle);

    }
    else
    {
        printf("semaphore did not create sucessfully\n");
	}

	if (xSemaphore_force_on !=NULL)
    {
        // creating tasks and their handles  
		TaskHandle_t xTaskForceOn = NULL;
		xTaskCreate(vTaskForceOn, "FORCE ON", 4096, NULL , FORCE_ON_PRIORITY , &xTaskForceOn);
    }
    else
    {
        printf("semaphore did not create sucessfully\n");
	}


	if (xSemaphore_force_off !=NULL)
    {
        // creating tasks and their handles  
		TaskHandle_t xTaskForceOff = NULL;
		xTaskCreate(vTaskForceOff, "FORCE OFF", 4096, NULL , FORCE_OFF_PRIORITY , &xTaskForceOff);
    }
    else
    {
        printf("semaphore did not create sucessfully\n");
	}

	if (xSemaphore_pulse !=NULL)
    {
        // creating tasks and their handles  
		TaskHandle_t xTaskPulse = NULL;
		xTaskCreate(vTaskPulse, "PULSE", 4096, NULL , PULSE_PRIORITY , &xTaskPulse);
    }
    else
    {
        printf("semaphore did not create sucessfully\n");
	}
	if (xSemaphore_send !=NULL)
    {
        // creating tasks and their handles  
        TaskHandle_t xTaskWatch = NULL;
		xTaskCreate(vTaskWatch, "WATCH", 4096, NULL , WATCH_PRIORITY , &xTaskWatch);
		xTaskCreate(udp_client_task, "UDP_CLIENT", 4096, (void*)AF_INET , SEND_PRIORITY , NULL);
    }
    else
    {
        printf("semaphore did not create sucessfully\n");
	}

}
