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
#include <stdint.h>
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
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "ethernet_init.h"
#include "msgpack.h"
#include "wifi.h"


#define PORT CONFIG_PORT
#define PORT_2 CONFIG_PORT_2
#define STATIC_IP_ADDR        CONFIG_STATIC_IP_ADDR
#define STATIC_NETMASK_ADDR   CONFIG_STATIC_NETMASK_ADDR
#define JUICE_GPIO  CONFIG_GPIO_JUICE
#define LIGHT_ON_GPIO  CONFIG_GPIO_LIGHT_ON
#define LIGHT_OFF_GPIO  CONFIG_GPIO_LIGHT_OFF
// TODO make this programmatic?
#define NUM_RESOURCES 4 // 3 GPIO
#define CLIENT_PRIORITY (tskIDLE_PRIORITY + 4)
#define SERVER_PRIORITY (tskIDLE_PRIORITY + 10)
#define KILL_PRIORITY  (tskIDLE_PRIORITY + 3)
#define LEDC_PRIORITY (tskIDLE_PRIORITY + 4)
#define GPIO_PRIORITY  (tskIDLE_PRIORITY + 5)
#define POLARIS_PRIORITY (tskIDLE_PRIORITY + 4)
#define TIMEOUT_SOCKET_SEC CONFIG_TIMEOUT_SOCKET_SEC
#define TIMEOUT_SOCKET_USEC CONFIG_TIMEOUT_SOCKET_USEC
#define HOST_IP_ADDR CONFIG_HOST_IP_ADDR // computer's ip address communicating with lico   
#define QUEUE_SIZE 3  // Modify to save on application memory, as well as stack size for each task
#define GPIO_MAX_SEQ 5 // maximum number of gpio action modules you can put together (arbitrarily defined for now)
#define MAX_BUF_LEN 1500
#define POLARIS_CMD_MAX_LEN 15
#define FREERTOS_HZ CONFIG_FREERTOS_HZ 
#define N_GPIO_TASKS 10 // can support up to around 20 max given application memory, and I fine tuned the stack size for each task

// analog read
#define ADC_ATTEN ADC_ATTEN_DB_12

static const char *TAG = "eth_example";
char input;
static int inactive_tasks[N_GPIO_TASKS];
static int avail_task = -1;
int task_index;
int t;

static QueueHandle_t kill_queue = NULL;
static QueueHandle_t ledc_queue = NULL;
static QueueHandle_t udp_client_queue = NULL;
static QueueHandle_t gpio_queue[N_GPIO_TASKS];

typedef struct {
    int gpio;
    int val;
} Client_payload;

typedef struct {
    int addr_family;
    char *ip_addr;
    int port;
} Server_params;

typedef struct {
    char recbytes[1500];
    int len;
} Rx_buf;

typedef struct {
    char* name;
    int duration;
    int delay_pre;
    int delay_post;
    int repeat;
} Action;

typedef struct {
    int gpio;
    int num_actions;
//    int tasknum;
    Action *act_seq[GPIO_MAX_SEQ]; // an unspecified array of pointers to the module structs
} Op_gpio;

typedef struct {
    int gpio;
    int freq;
    int duty;
//    int tasknum;
} Op_ledc;

int killtasknum = 0;
int ledctask = 0;

static void ledc_init(void){
	ledc_timer_config_t ledc_timer = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.timer_num  = LEDC_TIMER_0,
		.duty_resolution = LEDC_TIMER_13_BIT,
		.freq_hz = 1,
		.clk_cfg = LEDC_AUTO_CLK
	};
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
	
	ledc_channel_config_t ledc_channel = {
		.speed_mode     = LEDC_LOW_SPEED_MODE,
		.channel        = LEDC_CHANNEL_0,
		.timer_sel      = LEDC_TIMER_0,
		.intr_type      = LEDC_INTR_DISABLE,
		.gpio_num       = 5,
		.duty           = 4095, //duty * ((2**13)-1), default set to 50%
		.hpoint         = 0 //high mode counter
	};
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}


// parse the keys for a custom square wave task
void parse_keys(char* keys[], msgpack_object_kv* map_ptr, Op_gpio *op){  // the key ordering/structure in the dict is sensitive; i.e. in the dict sent to wESP32, the pin field has to be first key before action_seq
    Action *action;

    ESP_LOGI(TAG, "%s",keys[0]);
	
    if (strstr(keys[0], "gpio")!=0) { // if first entry in dict is a gpio pin
        ESP_LOGI(TAG, "gpio task detected");

	op->gpio = map_ptr[0].val.via.i64;

        op->num_actions = map_ptr[1].val.via.array.size;
	
	//op->tasknum = map_ptr[2].val.via.i64;
	//ESP_LOGI(TAG, "task number = %d",op->tasknum);

        for (int i = 0; i < op->num_actions; i++) {

            action = malloc(sizeof(Action));
            op->act_seq[i] = action;

            msgpack_object_str act_str_obj= map_ptr[1].val.via.array.ptr[i].via.map.ptr[0].val.via.str; // 1st index (1) is for act_seq, 2nd index (a) is for action number, 3rd index (0) is for what config of the action, in this case "action"
            action->name = (char*) malloc((act_str_obj.size+1)*sizeof(char));

            for (int l=0; l<act_str_obj.size; l++) {
                action->name[l] = *(act_str_obj.ptr + l);
            }
            action->name[act_str_obj.size] = 0;

            action->duration = map_ptr[1].val.via.array.ptr[i].via.map.ptr[1].val.via.i64;
            action->delay_pre = map_ptr[1].val.via.array.ptr[i].via.map.ptr[2].val.via.i64;
            action->delay_post = map_ptr[1].val.via.array.ptr[i].via.map.ptr[3].val.via.i64;
            action->repeat = map_ptr[1].val.via.array.ptr[i].via.map.ptr[4].val.via.i64;
	}
	
    
    }

}

// parse the object for a custom square wave task
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

// parse the keys for a ledc task
void parse_keys_ledc(char* keys[], msgpack_object_kv* map_ptr, Op_ledc *op){
    ESP_LOGI(TAG, "%s",keys[0]);
	
    if (strstr(keys[0], "ledc")!=0) { // if first entry in dict is a gpio pin
        ESP_LOGI(TAG, "ledc task detected");

	op->gpio = map_ptr[0].val.via.i64;

	ledctask = op->gpio; //map_ptr[3].val.via.i64;

        op->freq = map_ptr[1].val.via.i64;
	
        op->duty = map_ptr[2].val.via.i64;
    
    }
}

// parse the object for a ledc task
void parse_object_ledc(msgpack_object obj, Op_ledc *op){
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

        parse_keys_ledc(keys, map_ptr, op);

    }
}

// parse the keys for a kill task
void parse_keys_kill(char* keys[], msgpack_object_kv* map_ptr){  // the key ordering/structure in the dict is sensitive; i.e. in the dict sent to wESP32, the pin field has to be first key before action_seq
    ESP_LOGI(TAG, "%s",keys[0]);
	
    
    if (strstr(keys[0],"killtasknum")!=0){ // if dict is a kill operation
	ESP_LOGI(TAG, "kill task detected");
	killtasknum = map_ptr[0].val.via.i64;
	if (killtasknum == ledctask) {
		ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0));
		killtasknum = 0;
	}
	ESP_LOGI(TAG, "killtasknum set to %d", killtasknum);
	   }
}

// parse the object for a kill task
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
    emac_config.smi_mdc_gpio_num = 23;
    emac_config.smi_mdio_gpio_num = 18;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);

    // Initialize phy config to default
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
#if CONFIG_ESP_WESP32
    phy_config.reset_gpio_num = -1;
    phy_config.phy_addr = 0;
    esp_eth_phy_t *phy = esp_eth_phy_new_rtl8201(&phy_config);
#elif CONFIG_ESP_WT32_ETH01
    phy_config.reset_gpio_num = 16;
    phy_config.phy_addr = 1;
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);
#endif

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
    Server_params *server_params = (Server_params *)pvParameters;
    int addr_family = server_params->addr_family;
    int ip_protocol = 0;
    struct sockaddr_in addr;
    int sock;
    printf("\n FREERTOS_HZ is : %d\n", FREERTOS_HZ);
    while (1) {

        addr.sin_addr.s_addr = inet_addr(server_params->ip_addr);  // htonl(INADDR_ANY);
        addr.sin_family = AF_INET;
        addr.sin_port = htons(server_params->port);
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
        ESP_LOGI(TAG, "Socket bound, port %d", server_params->port);

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
		

        		if(strstr(rx_buf.recbytes,"gpio")!= NULL){
                    avail_task = -1;
                    for (task_index=0; task_index<N_GPIO_TASKS; task_index++) { // find what is the next available free task
                        if (inactive_tasks[task_index] >= 0 ) { // if this task index is an inactive task
                            avail_task = task_index;
                        }
                    }
                    if (avail_task < 0) {
                        ESP_LOGE(TAG, "There are no more available GPIO workers to run");        
                    }
                    else {
                        printf("\nin udp_server_task, &rx_buf is %p\n", &rx_buf);
                        xQueueSend(gpio_queue[avail_task], &rx_buf, portMAX_DELAY);
                    }
                    printf("\nAdded to gpio task %d\n", avail_task);
                }    
        		else if(strstr(rx_buf.recbytes,"ledc")!= NULL){
        			xQueueSend(ledc_queue, &rx_buf, portMAX_DELAY);
        			ESP_LOGI(TAG, "added to ledc queue");
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

int gpio_to_adc_chan(int num, int gpio) {
    int chan = -1;
    if (num == 1) {
        if (gpio > 35) chan = gpio - 36;
        else chan = gpio - 28;

        if (chan < 0 || chan > 7) {
            printf("Unsupported ADC1 channel.\n");
            return -1;
        }
    }
    else if (num == 2) {
        // TODO support ESP32-S2 which has a different mapping
        // https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/api-reference/peripherals/adc.html#_CPPv414adc2_channel_t
        if (gpio == 4) chan = 0;
        else if (gpio == 0) chan = 1;
        else if (gpio == 2) chan = 2;
        else if (gpio == 15) chan = 3;
        else if (gpio == 13) chan = 4;
        else if (gpio == 12) chan = 5;
        else if (gpio == 14) chan = 6;
        else if (gpio == 27) chan = 7;
        else if (gpio == 25) chan = 8;
        else if (gpio == 26) chan = 9;

        if (chan < 0 || chan > 9) {
            printf("Unsupported ADC2 channel.\n");
            return -1;
        }
    }
    else {
        printf("Unsupported ADC number.\n");
        return -1;
    }
    return chan;
}

int adc_read_oneshot(int num, int chan) {
    return 1;
}

void unpack_ctrl_GPIO(Rx_buf rx_buf, int index) {
    inactive_tasks[index] = -1; // take this task off the inactive task array so it won't be called upon
    Op_gpio *op;
    op = malloc(sizeof(Op_gpio));
    int repeat_counter;
    int repeat;
    Action *action;
    msgpack_unpacked und;
    bool gpio_state = 0;

    // read
    int adc_num;
    Client_payload client_payload;
    int adc_chan;
    int read_val;

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
        printf("\nError in unpacking. Ret = %d\n", ret);        
    }
    printf("rx_buf is %s", rx_buf.recbytes);

    // Destroy unpacked object
    msgpack_unpacked_destroy(&und);

    for (int i = 0; i < op->num_actions; i++) { // for each action
	    repeat_counter = 0;
        repeat = 1;
        action = op->act_seq[i];

        printf("freeRTOS tick length: %ld \n",portTICK_PERIOD_MS);
        printf("repeat is %d\n", action->repeat);

        while(repeat && (op->gpio != killtasknum)) { // for each repeat

            // pre-delay
            if (action->delay_pre > 0) {
              printf("delaying before pulse for %d msec\n",action->delay_pre);
              vTaskDelay(action->delay_pre / portTICK_PERIOD_MS);
            }

            // action
            if (!strncmp(action->name, "read", 4)) {
                client_payload.gpio = op->gpio;
                if(!strcmp(action->name, "read_digital")) {
                    printf("reading gpio %dn", op->gpio);

                    // TODO add error handling to make sure pin is in input mode
                    read_val = gpio_get_level(op->gpio);
                }
                else if(!strcmp(action->name, "read_analog")) {
                    printf("reading adc %dn", op->gpio);

                    adc_num = 2;
                    if (op->gpio >= 36) {
                        adc_num = 1;
                    }
                    adc_chan = gpio_to_adc_chan(adc_num, op->gpio);

                    if (action->repeat == 1) {
                        read_val = adc_read_oneshot(adc_num, adc_chan);
                    }
                    else {
                        printf("Defaulting to oneshot. Continuous analog read not supported.");
                        // TODO implement continuous read
                        read_val = adc_read_oneshot(adc_num, adc_chan);
                    }

                    client_payload.val = read_val;
                    xQueueSend(udp_client_queue, &client_payload, portMAX_DELAY);
                }
            }
            else {
                if (!strcmp(action->name, "up")){
                    gpio_state = 1;
                    printf(
                        "setting gpio %d up for %d msec\n",
                        op->gpio, action->duration
                    );
                }
                else if (!strcmp(action->name, "down")) {
                    gpio_state = 0;
                    printf(
                        "setting gpio %d down for %d msec\n",
                        op->gpio, action->duration
                    );
                }
                else {
                    printf(
                        "Skipping unsupported action: %s\n",
                        action->name
                    );
                    continue;
                }

                gpio_set_level(op->gpio, gpio_state);
                if (action->duration > 0) { // if duration is defined, then this is a pulse for the duration, else the gpio stays in the state
                    vTaskDelay(action->duration / portTICK_PERIOD_MS);
                    gpio_set_level(op->gpio, !gpio_state);
                }
            }


            // post-delay
            if (action->delay_post > 0) {
                printf("delaying after pulse for %d msec\n",action->delay_post);
                vTaskDelay(action->delay_post / portTICK_PERIOD_MS);
            }
               
            // check repeat
            repeat_counter++;
            if (repeat_counter == action->repeat) {
                repeat = 0;
            }
	    }
    }
    killtasknum = 0;
    free(op);
    printf("\nGPIO task %d is done\n", index);
    inactive_tasks[index] = index; // once this func is done, put this task back on the inactive task array
}


void vTaskGPIO(void *pvParameters) {
    vTaskSetThreadLocalStoragePointer(NULL, 0, (void *) t); // keep the memory of the task index to each task's local memory
    int index = (int) pvTaskGetThreadLocalStoragePointer(NULL, 0);
    Rx_buf rx_buf;
    for (;;) {
	    if (xQueueReceive(gpio_queue[index], &rx_buf, portMAX_DELAY)) {
            printf("\n In vTaskGPIO &rx_buf is %p \n" , &rx_buf);
            unpack_ctrl_GPIO(rx_buf, index);
        }
    }
}



// task definition for ledc task
void vTaskLEDC(void * pvParameters){
    Op_ledc *op;
    Rx_buf rx_buf;
    op = malloc(sizeof(Op_ledc));
    for (;;) {
	if (xQueueReceive(ledc_queue, &rx_buf, portMAX_DELAY)) {
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
                parse_object_ledc(object, op);
            }
            else {
                printf("Error in unpacking");
            }

            // Destroy unpacked object
            msgpack_unpacked_destroy(&und);

            printf("freeRTOS tick length: %ld \n",portTICK_PERIOD_MS);
	    int gpio = op->gpio;	
	    int freq = op->freq;
	    int dutyfrac = op->duty;
	    
	    int duty = ((8192*dutyfrac)/100) - 1;

	    printf("duty is %d \n", duty);
	    printf("freq is %d \n",freq);	

	    ledc_init();
	    	
	    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,duty));
	    	
	    ESP_ERROR_CHECK(ledc_set_pin(gpio,LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0));
	    	
	    ESP_ERROR_CHECK(ledc_set_freq(LEDC_LOW_SPEED_MODE,LEDC_TIMER_0,freq));
	    	
	    //if (repeat > 0) {
	    // 	ESP_ERROR_CHECK(ledc_set_duty_with_hpoint(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,duty,repeat));
	    //}
	    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0));
        }
    }
    free(op);
}

// task definition for kill task
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

// UDP client task function
void udp_client_task( void * pvParameters ) {
    ESP_LOGI(TAG, "Client task started");
    int addr_family = 0;
    int ip_protocol = 0;
    int sock;
    struct sockaddr_in dest_addr;
    Client_payload client_payload;

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
        if (xQueueReceive(udp_client_queue, &client_payload, portMAX_DELAY)) {
            int err = sendto(
                sock, &client_payload, sizeof(Client_payload), 0,
                (struct sockaddr *)&dest_addr, sizeof(dest_addr)
            );
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                return;
            }
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

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

static void configure_analog(
    adc_oneshot_unit_handle_t *handle,
    adc_oneshot_unit_init_cfg_t *init_config,
    adc_cali_handle_t adc_cali_chan_handle
) {
    //-------------ADC1 Init---------------//
    ESP_ERROR_CHECK(adc_oneshot_new_unit(init_config, handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*handle, ADC_CHANNEL_0, &config));

    //-------------ADC1 Calibration Init---------------//
    // TODO loop over all channels enabled and handle return value
    example_adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_0, ADC_ATTEN, &adc_cali_chan_handle);
}

static void teardown_analog(
    adc_oneshot_unit_handle_t *handle,
    adc_cali_handle_t *adc_cali_chan_handle
) {
    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(*handle));
    // TODO loop over all channels enabled
    example_adc_calibration_deinit(*adc_cali_chan_handle);
}

void app_main(void) {
#if CONIFG_USE_INTERNAL_ETHERNET
    init_wesp32_eth();
#endif
#if CONFIG_USE_WIFI
    wifi_init_sta(true);
    // example_set_static_ip(eth_netif);
#endif


    Server_params sp1;
    Server_params sp2;

    sp1.addr_family = AF_INET;
    sp1.port = PORT;
    sp1.ip_addr = STATIC_IP_ADDR;

    sp2.addr_family = AF_INET;
    sp2.port = PORT_2;
    sp2.ip_addr = STATIC_IP_ADDR;

    xTaskCreate(udp_server_task, "UDP_SERVER", 1024*4, (void*)&sp1, SERVER_PRIORITY, NULL);
    xTaskCreate(udp_server_task, "UDP_SERVER_2", 1024*4, (void*)&sp2, SERVER_PRIORITY, NULL);

    configure_led();

    // TODO configure all enabled channels across ADCs
    adc_oneshot_unit_handle_t adc_handle_1;
    adc_oneshot_unit_init_cfg_t adc_init_config_1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_cali_handle_t adc_cali_chan_handle_1 = NULL;
    configure_analog(&adc_handle_1, &adc_init_config_1, adc_cali_chan_handle_1);


    // create event queues
    if (
        !(udp_client_queue = xQueueCreate(QUEUE_SIZE, sizeof(Client_payload)))
    ) {
        ESP_LOGE(TAG, "Could not create UDP client queue.");
    }
    if (!(kill_queue = xQueueCreate(QUEUE_SIZE, sizeof(Rx_buf)))) {
        ESP_LOGE(TAG, "Could not create kill queue.");
    }
 
    if (!(ledc_queue = xQueueCreate(QUEUE_SIZE, sizeof(Rx_buf)))) {
        ESP_LOGE(TAG, "Could not create ledc queue.");
    }


    // create tasks and their handles
    for (t=0; t<N_GPIO_TASKS; t++) { // creating many instances of gpio tasks, one queue for each task
        inactive_tasks[t] = t;  // initialize list of inactive tasks
        if (!(gpio_queue[t] = xQueueCreate(QUEUE_SIZE, sizeof(Rx_buf)))) {
            ESP_LOGE(TAG, "Could not create GPIO Queue");
        }
        xTaskCreate(vTaskGPIO, NULL, 1024*6, &t , GPIO_PRIORITY , NULL);
    }
    
    TaskHandle_t xTaskKill = NULL;
    xTaskCreate(vTaskKill, "KILL", 1024*8, NULL , KILL_PRIORITY , &xTaskKill);
    
    TaskHandle_t xTaskLEDC = NULL;
    xTaskCreate(vTaskLEDC, "LEDC", 4096, NULL, LEDC_PRIORITY, &xTaskLEDC);
    
    xTaskCreate(udp_client_task, "UDP_CLIENT", 1024*2, (void*)AF_INET , CLIENT_PRIORITY , NULL);

    // TODO where to put this?
    // teardown_analog(&adc_handle_1, &adc_init_config_1, adc_cali_chan_handle_1);
}
