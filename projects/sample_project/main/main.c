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
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
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
#include "msgpack.h"

#include "polaris.h"

#include <stdint.h>


#define PORT CONFIG_EXAMPLE_PORT
#define STATIC_IP_ADDR        CONFIG_EXAMPLE_STATIC_IP_ADDR
#define STATIC_NETMASK_ADDR   CONFIG_EXAMPLE_STATIC_NETMASK_ADDR
#define JUICE_GPIO  CONFIG_GPIO_JUICE
#define LIGHT_ON_GPIO  CONFIG_GPIO_LIGHT_ON
#define LIGHT_OFF_GPIO  CONFIG_GPIO_LIGHT_OFF
// TODO make this programmatic?
#define NUM_RESOURCES 4 // 3 GPIO, and polaris TX
#define CLIENT_PRIORITY (tskIDLE_PRIORITY + 9)
#define SERVER_PRIORITY (tskIDLE_PRIORITY + 10)
#define KILL_PRIORITY  (tskIDLE_PRIORITY + 3)
#define GPIO_PRIORITY  (tskIDLE_PRIORITY + 5)
#define POLARIS_PRIORITY (tskIDLE_PRIORITY + 4)
#define PULSE_LENGTH_MSEC CONFIG_PULSE_LENGTH_MSEC
#define TIMEOUT_SOCKET_SEC CONFIG_TIMEOUT_SOCKET_SEC
#define TIMEOUT_SOCKET_USEC CONFIG_TIMEOUT_SOCKET_USEC
#define HOST_IP_ADDR CONFIG_HOST_IP_ADDR // computer's ip address communicating with lico   
#define QUEUE_SIZE 10
#define GPIO_MAX_SEQ 5 // maximum number of gpio action modules you can put together (arbitrarily defined for now)
#define MAX_BUF_LEN 1500
#define POLARIS_CMD_MAX_LEN 15
#define CONFIG_FREERTOS_HZ 1000

static const char *TAG = "eth_example";
char input;

static QueueHandle_t kill_queue = NULL;
static QueueHandle_t udp_client_queue = NULL;
static QueueHandle_t gpio_queue = NULL;
static QueueHandle_t polaris_queue = NULL;

typedef struct {
    char recbytes[1500];
    int len;
} Rx_buf;

typedef struct {
    int gpio;
    int num_actions;
    int tasknum;
    struct module {
        char* action;
        int duration;
        int delay_pre;
        int delay_post;
        int64_t repeat;
    } *act_seq[GPIO_MAX_SEQ]; // an unspecified array of pointers to the module structs
} Op_gpio;

int killtasknum = 0;

void parse_keys(char* keys[], msgpack_object_kv* map_ptr, Op_gpio *op){  // the key ordering/structure in the dict is sensitive; i.e. in the dict sent to wESP32, the pin field has to be first key before action_seq
    ESP_LOGI(TAG, "%s",keys[0]);
	
    if (strstr(keys[0], "gpio")!=0) { // if first entry in dict is a gpio pin
        ESP_LOGI(TAG, "gpio task detected");

	op->gpio = map_ptr[0].val.via.i64;

        op->num_actions = map_ptr[1].val.via.array.size;
	
	op->tasknum = map_ptr[2].val.via.array.size;

        for (int a = 0 ; a < op->num_actions; a++) {

            op->act_seq[a] = malloc(sizeof(struct module));

            msgpack_object_str act_str_obj= map_ptr[1].val.via.array.ptr[a].via.map.ptr[0].val.via.str; // 1st index (1) is for act_seq, 2nd index (a) is for action number, 3rd index (0) is for what config of the action, in this case "action"
            op->act_seq[a]->action = (char*) malloc((act_str_obj.size+1)*sizeof(char));

            for (int l=0; l<act_str_obj.size; l++) {
                op->act_seq[a]->action[l] = *(act_str_obj.ptr + l);
            }
            op->act_seq[a]->action[act_str_obj.size] = 0;

            op->act_seq[a]->duration = map_ptr[1].val.via.array.ptr[a].via.map.ptr[1].val.via.i64;
            op->act_seq[a]->delay_pre = map_ptr[1].val.via.array.ptr[a].via.map.ptr[2].val.via.i64;
            op->act_seq[a]->delay_post = map_ptr[1].val.via.array.ptr[a].via.map.ptr[3].val.via.i64;
            op->act_seq[a]->repeat = map_ptr[1].val.via.array.ptr[a].via.map.ptr[4].val.via.i64;		
	}
	
    
    }

}


void parse_object(msgpack_object obj, Op_gpio *op) {
    if (obj.type == 7) {  // if the deserialized data is dict/map
        uint32_t map_size = obj.via.map.size;
        msgpack_object_kv* map_ptr = obj.via.map.ptr; // pointer to msgpack_object_map
        char* keys[map_size];
        for (int n=0; n<map_size; n++) { // for each key/value pair in the dict, get the list of keys
            msgpack_object_str key_str_obj = map_ptr[n].key.via.str; // get msgpack_object_str
            int keysize = key_str_obj.size;
            char* key_str = (char*) malloc((keysize+1)*sizeof(char));
            for (int j=0; j<keysize; j++) { // extract out byte by byte the key bc strcpy doesn't work
                key_str[j] = *(key_str_obj.ptr+j);
            }
            key_str[keysize] = 0;
            keys[n] = key_str;
        }

        parse_keys(keys, map_ptr, op);

    }

}

void parse_keys_kill(char* keys[], msgpack_object_kv* map_ptr){  // the key ordering/structure in the dict is sensitive; i.e. in the dict sent to wESP32, the pin field has to be first key before action_seq
    ESP_LOGI(TAG, "%s",keys[0]);
	
    
    if (strstr(keys[0],"killtasknum")!=0){ // if dict is a kill operation
	ESP_LOGI(TAG, "kill task detected");
	killtasknum = map_ptr[0].val.via.i64;
	ESP_LOGI(TAG, "killtasknum set to %d", killtasknum);
	   }
}


void parse_object_kill(msgpack_object obj) {
    if (obj.type == 7) {  // if the deserialized data is dict/map
        uint32_t map_size = obj.via.map.size;
        msgpack_object_kv* map_ptr = obj.via.map.ptr; // pointer to msgpack_object_map
        char* keys[map_size];
        for (int n=0; n<map_size; n++) { // for each key/value pair in the dict, get the list of keys
            msgpack_object_str key_str_obj = map_ptr[n].key.via.str; // get msgpack_object_str
            int keysize = key_str_obj.size;
            char* key_str = (char*) malloc((keysize+1)*sizeof(char));
            for (int j=0; j<keysize; j++) { // extract out byte by byte the key bc strcpy doesn't work
                key_str[j] = *(key_str_obj.ptr+j);
            }
            key_str[keysize] = 0;
            keys[n] = key_str;
        }

        parse_keys_kill(keys, map_ptr);

    }

}

/* Event handler for Ethernet events */
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


static void udp_server_task(void *pvParameters) {
    Rx_buf rx_buf;
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

        int err = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);


        while (1) {
            ESP_LOGI(TAG, "Waiting for data...");
            rx_buf.len = recvfrom(
                sock, rx_buf.recbytes, sizeof(rx_buf.recbytes), 0,
                (struct sockaddr *)&source_addr, &socklen
            );
            // Error occurred during receiving
            if (rx_buf.len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                ESP_LOGI(TAG, "Received %d bytes.", rx_buf.len);

                rx_buf.recbytes[rx_buf.len] = '\0'; // Null-terminate buffer
                
                ESP_LOGI(TAG, "%s", rx_buf.recbytes);
		
		if (strncmp(rx_buf.recbytes, "polaris", 7) == 0) {
                    xQueueSend(polaris_queue, rx_buf.recbytes, portMAX_DELAY);
		}
		else if(strstr(rx_buf.recbytes,"gpio")!= NULL){
                    xQueueSend(gpio_queue, &rx_buf, portMAX_DELAY);
		    ESP_LOGI(TAG, "added to gpio queue");
                }
		else if(strstr(rx_buf.recbytes,"killtasknum")!= NULL) {
		    xQueueSend(kill_queue, &rx_buf, portMAX_DELAY);
		    ESP_LOGI(TAG, "added to kill queue");

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

void vTaskGPIO(void * pvParameters) {
    Op_gpio *op;
    Rx_buf rx_buf;
    op = malloc(sizeof(Op_gpio));
    for (;;) {
        if (xQueueReceive(gpio_queue, &rx_buf, portMAX_DELAY)) {
            msgpack_unpacked und;
            // Initalize unpacked object
            msgpack_unpacked_init(&und);
            //Unpack
            msgpack_unpack_return ret;
            ret = msgpack_unpack_next(&und, rx_buf.recbytes, rx_buf.len, NULL);
            if (ret == MSGPACK_UNPACK_SUCCESS) {
                msgpack_object object = und.data;
                msgpack_object_print(stdout, object);
                printf("\n");
                // Do stuff with unpacked object...
                parse_object(object, op);
            }
            else {
                printf("Error in unpacking");
            }

            // Destroy unpacked object
            msgpack_unpacked_destroy(&und);

	    for (int a = 0; a < op->num_actions; a++) { // for each action
                printf("freeRTOS tick length: %ld \n",portTICK_PERIOD_MS);
		
		int gpio = op->gpio;
		int repeat = op->act_seq[a]->repeat;
		int delay_pre = op->act_seq[a]->delay_pre;
		int duration = op->act_seq[a]->duration;
		int delay_post = op->act_seq[a]->delay_post;
		int tasknum = op->tasknum;

		// if given up action:
		if (!strcmp(op->act_seq[a]->action,"up")){
		}

            }
        }
    }
    free(op);
}
		
void vTaskKill(void * pvParameters) {
    Rx_buf rx_buf;
    
    for (;;) {
        if (xQueueReceive(kill_queue, &rx_buf, portMAX_DELAY)) {
            msgpack_unpacked und;

            // Initalize unpacked object
            msgpack_unpacked_init(&und);
            
	    //Unpack
            msgpack_unpack_return ret;
            ret = msgpack_unpack_next(&und, rx_buf.recbytes, rx_buf.len, NULL);
            
	    if (ret == MSGPACK_UNPACK_SUCCESS) {
                msgpack_object object = und.data;
                msgpack_object_print(stdout, object);
                printf("\n");
                // Do stuff with unpacked object...
                parse_object_kill(object);
            }
            else {
                printf("Error in unpacking");
            }

            // Destroy unpacked object
            msgpack_unpacked_destroy(&und);
        }
    }
}	

void vTaskPolaris(void *pvParameters) {
    char rx_buffer[POLARIS_CMD_MAX_LEN];
    char *polaris_read_buf;
    Polaris_frame frame;
    polaris_read_buf = malloc(32768);
    int streaming = 0;

    for (;;) {
        if(streaming || xQueueReceive(polaris_queue, rx_buffer, portMAX_DELAY)) {
            if (streaming) {
                polaris_read(&frame, polaris_read_buf);
                xQueueSend(udp_client_queue, &frame, portMAX_DELAY);
                if (strlen(rx_buffer) == 0) {
                    continue;
                }
            }

            if (strcmp(rx_buffer, "polaris_init") == 0) {
                streaming = 0;

                // initialize polaris with correct baud rates
                polaris_set_baudrate(9600); // in case of re-init
                polaris_send_init_seq(polaris_read_buf);
                polaris_set_baudrate(115200);

                // read and send frame as ACK
                polaris_read(&frame, polaris_read_buf);
                xQueueSend(udp_client_queue, &frame, portMAX_DELAY);

                rx_buffer[0] = '\0';
            }
            else if (strcmp(rx_buffer, "polaris_stream") == 0) {
                streaming = 1;
                rx_buffer[0] = '\0';
            }
            else if (strcmp(rx_buffer, "polaris_down") == 0) {
                streaming = 0;
                rx_buffer[0] = '\0';
            }
            else {
                ESP_LOGE(TAG, "Unknown polaris command.");
            }
        }
    }
}

// UDP client task function
void udp_client_task( void * pvParameters ) {
    ESP_LOGI(TAG, "Client task started");
    int addr_family = 0;
    int ip_protocol = 0;
    int sock;
    struct sockaddr_in dest_addr;

    Polaris_frame polaris_frame;

    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }

    ESP_LOGI(TAG, "Client Socket created");

    for (;;) {
        if (xQueueReceive(udp_client_queue, &polaris_frame, portMAX_DELAY)) {
            int err = sendto(
                sock, &polaris_frame, 16, 0,
                (struct sockaddr *)&dest_addr, sizeof(dest_addr)
            );
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                return;
            }
            ESP_LOGI(TAG, "Polaris frame sent.");
        }
    }
}

static void configure_led(void) {
    gpio_reset_pin(JUICE_GPIO);
    gpio_set_direction(JUICE_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LIGHT_ON_GPIO);
    gpio_set_direction(LIGHT_ON_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LIGHT_OFF_GPIO);
    gpio_set_direction(LIGHT_OFF_GPIO, GPIO_MODE_OUTPUT);
}

void app_main(void) {
    init_wesp32_eth();

    printf("Timer: %lld us\n", esp_timer_get_time());

    xTaskCreate(udp_server_task, "UDP_SERVER", 8192, (void*)AF_INET, SERVER_PRIORITY, NULL);

    configure_led();

    polaris_setup_uart(9600);

    // create event queues
    if (!(udp_client_queue = xQueueCreate(QUEUE_SIZE, sizeof(Polaris_frame)))) {
        ESP_LOGE(TAG, "Could not create UDP client queue.");
    }
    if (!(gpio_queue = xQueueCreate(QUEUE_SIZE, sizeof(Rx_buf)))) {
        ESP_LOGE(TAG, "Could not create GPIO queue.");
    }

    if (!(kill_queue = xQueueCreate(QUEUE_SIZE, sizeof(Rx_buf)))) {
        ESP_LOGE(TAG, "Could not create kill queue.");
    }
 
    if (!(polaris_queue = xQueueCreate(QUEUE_SIZE, sizeof(char) * POLARIS_CMD_MAX_LEN))) {
        ESP_LOGE(TAG, "Could not create polaris queue.");
    }

    // create mutexes for multiple worker threads?
    for (int i = 0; i < NUM_RESOURCES; i++) {

    }
      

    // create tasks and their handles
    TaskHandle_t xTaskGPIO = NULL;
    xTaskCreate(vTaskGPIO, "TOGGLE", 4096, NULL , GPIO_PRIORITY , &xTaskGPIO);
    
    TaskHandle_t xTaskKill = NULL;
    xTaskCreate(vTaskKill, "KILL", 4096, NULL , KILL_PRIORITY , &xTaskKill);
    
    TaskHandle_t xTaskPolaris = NULL;
    xTaskCreate(vTaskPolaris, "POLARIS", 4096, NULL , POLARIS_PRIORITY , &xTaskPolaris);
    
    xTaskCreate(udp_client_task, "UDP_CLIENT", 4096, (void*)AF_INET , CLIENT_PRIORITY , NULL);
}
