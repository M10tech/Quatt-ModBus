/* (c) 2024 M10tech
 * Modbus-sniffer for ESP32
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"
#include "esp_netif_sntp.h"
// #include "lcm_api.h"
#include <udplogger.h>
#include "driver/uart.h"
#include "soc/uart_reg.h"
#include "ping/ping_sock.h"

// You must set version.txt file to match github version tag x.y.z for LCM4ESP32 to work

char    *pinger_target=NULL;

uint32_t calccrc(uint8_t * data, int len) { //Modbus CRC algorithm
    uint32_t crc = 0xFFFF;

    for (int pos = 0; pos < len; pos++) {
        crc ^= (int)data[pos];         // XOR byte into least sig. byte of crc
        for (int i = 8; i != 0; i--) {  // Loop over each bit
            if ((crc & 0x0001) != 0) {  // If the LSB is set
                crc >>= 1;              // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else  crc >>= 1;          // Else LSB is not set so Just shift right
        }
    }
    return crc; // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
}

#define RD_BUF_SIZE (1024)
static QueueHandle_t uart_queue;
static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    size_t msg_len;
    uint8_t* message = (uint8_t*) malloc(RD_BUF_SIZE);
    uint32_t msg_crc,datacrc; //*data=NULL,
    bool read_match=false;
    uint8_t x0f9f_addr[]={1,6,0x0f,0x9f},x0f9f[2];
    uint8_t x07cf_addr[]={1,6,0x07,0xcf},x07cf[2];
    uint8_t x07da_addr[]={1,6,0x07,0xda},x07da[2];
    uint8_t x07df_addr[]={1,6,0x07,0xdf},x07df[2];
    uint8_t x0833_read[]={1,3,0x08,0x33,0x00,0x28};
    uint8_t x0833_resp[]={1,3,0x50}; //TODO: make better match system
    while (true) {
        //Waiting for UART event.
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            char strtm[32];
            struct timeval tv;
            gettimeofday(&tv, NULL);
            time_t now=tv.tv_sec;
            struct tm *tm = localtime(&now);
            sprintf(strtm,"%2d_%02d:%02d:%02d",tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec);
            bzero(message, RD_BUF_SIZE);
            switch (event.type) {
            //Event of UART receving data
            case UART_DATA:
                msg_len = uart_read_bytes(UART_NUM_1, message, event.size, 100);
                if (msg_len>0) {
                    datacrc=message[msg_len-1]*256+message[msg_len-2]; //swap bytes
                    msg_crc=calccrc(message,msg_len-2); //calc CRC
                    if (msg_crc==datacrc) { // compare and proces good CRC
                        if (!memcmp(message, x0f9f_addr, 4)) {x0f9f[0]=message[4];x0f9f[1]=message[5]; break;}
                        if (!memcmp(message, x07cf_addr, 4)) {x07cf[0]=message[4];x07cf[1]=message[5]; break;}
                        if (!memcmp(message, x07da_addr, 4)) {x07da[0]=message[4];x07da[1]=message[5]; break;}
                        if (!memcmp(message, x07df_addr, 4)) {x07df[0]=message[4];x07df[1]=message[5]; break;}
                        if (!memcmp(message, x0833_read, 6)) {read_match=true; break;} //ignore the read request
                        if (read_match && !memcmp(message, x0833_resp, 3)) { //dump all known entries
                            UDPLUS("OLD %s ",strtm);
                            for (int i=3;i<msg_len-2;i+=2) UDPLUS("%02x%02x ",message[i],message[i+1]);
                            UDPLUS("%02x%02x ",x0f9f[0],x0f9f[1]);
                            UDPLUS("%02x%02x ",x07cf[0],x07cf[1]);
                            UDPLUS("%02x%02x ",x07da[0],x07da[1]);
                            UDPLUS("%02x%02x ",x07df[0],x07df[1]);
                            UDPLUS("\n");
                            read_match=false;
                            break;
                        }
                        UDPLUS("NEW %4d",msg_len);
                        for (int i=0;i<msg_len-2;i++) UDPLUS(" %02x",message[i]);
                        UDPLUS("  CRC_OK\n");
                    } else { // bad CRC
                        UDPLUS("BAD %4d",msg_len);
                        for (int i=0;i<msg_len-2;i++) UDPLUS(" %02x",message[i]);
                        UDPLUS(" calccrc=%04lx != datacrc=%04lx\n",msg_crc,datacrc);
                    }
                }
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                UDPLUS("ring buffer full with event.size=%d\n",event.size);
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_1);
                xQueueReset(uart_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                msg_len = uart_read_bytes(UART_NUM_1, message, event.size, 100);
                if (msg_len>0) {UDPLUS("B%4d", msg_len); for (int i=0;i<msg_len;i++) UDPLUS(" %02x",message[i]); UDPLUS("\n");}
                break;
            default:
                UDPLUS("uart event type: %d and event size=%d\n", event.type, event.size);
                break;
            }
        }
    }
    free(message);
    message = NULL;
    vTaskDelete(NULL);
} 

int ping_count=60,ping_delay=1; //seconds
static void ping_success(esp_ping_handle_t hdl, void *args) {
    ping_count+=20; ping_delay+=5;
    if (ping_count>120) ping_count=120;
    if (ping_delay>60)  ping_delay=60;
    uint32_t elapsed_time;
    ip_addr_t response_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_time, sizeof(elapsed_time));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR,  &response_addr,  sizeof(response_addr));
    UDPLUS("good ping from %s %lu ms -> count: %d s\n", inet_ntoa(response_addr.u_addr.ip4), elapsed_time, ping_count);
}
static void ping_timeout(esp_ping_handle_t hdl, void *args) {
    ping_count--; ping_delay=1;
    UDPLUS("failed ping -> count: %d s\n", ping_count);
}
void ping_task(void *argv) {
    ip_addr_t target_addr;
    ipaddr_aton(pinger_target,&target_addr);
    esp_ping_handle_t ping;
    esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
    ping_config.target_addr = target_addr;
    ping_config.timeout_ms = 900; //should time-out before our 1s delay
    ping_config.count = 1; //one ping at a time means we can regulate the interval at will
    esp_ping_callbacks_t cbs = {.on_ping_success=ping_success, .on_ping_timeout=ping_timeout, .on_ping_end=NULL, .cb_args=NULL}; //set callback functions
    esp_ping_new_session(&ping_config, &cbs, &ping);
    
    UDPLUS("Pinging IP %s\n", ipaddr_ntoa(&target_addr));
    while(ping_count){
        vTaskDelay((ping_delay-1)*(1000/portTICK_PERIOD_MS)); //already waited 1 second...
        esp_ping_start(ping);
        vTaskDelay(1000/portTICK_PERIOD_MS); //waiting for answer or timeout to update ping_delay value
    }
    UDPLUS("restarting because can't ping home-hub\n");
    vTaskDelay(1000/portTICK_PERIOD_MS); //allow UDPlog to flush output
    esp_restart(); //TODO: disable GPIO outputs
}

void init_task(void *argv) {
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1); tzset();
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&config);
    while (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK) {
        UDPLUS("Still waiting for system time to sync\n");
    }
    time_t ts = time(NULL);
    UDPLUS("TIME SET: %u=%s\n", (unsigned int) ts, ctime(&ts));
    vTaskDelete(NULL);
}

void main_task(void *arg) {
    udplog_init(3);
    vTaskDelay(300); //Allow Wi-Fi to connect
    UDPLUS("\n\nModBusSniffer\n");
    const uart_port_t uart_num = UART_NUM_1;
    const int uart_buffer_size = (1024 * 2);
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, 0, 20, &uart_queue, 0));
    uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config)); // Configure UART parameters
    uart_intr_config_t uart_intr = {
        .intr_enable_mask = UART_RXFIFO_FULL_INT_ENA_M | UART_RXFIFO_TOUT_INT_ENA_M, //?? |UART_INTR_RXFIFO_TOUT,
        .rxfifo_full_thresh = 255,
        .rx_timeout_thresh = 3,
    };
    ESP_ERROR_CHECK(uart_intr_config(uart_num, &uart_intr));
    ESP_ERROR_CHECK(uart_enable_rx_intr(uart_num)); // Enable UART RX FIFO full threshold and timeout interrupts
    
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 1, 0, 2, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(uart_num,UART_MODE_RS485_HALF_DUPLEX));
    xTaskCreate(init_task,"Time", 2048, NULL, 6, NULL);
    pinger_target="192.168.178.5";
    xTaskCreate(ping_task,"PingT",2048, NULL, 1, NULL);
    xTaskCreate(uart_event_task,"uart",2048,NULL,12,NULL);
    
    while (true) {
        vTaskDelay(1000); 
    }
}    

void app_main(void) {
    printf("app_main-start\n");

    //The code in this function would be the setup for any app that uses wifi which is set by LCM
    //It is all boilerplate code that is also used in common_example code
    esp_err_t err = nvs_flash_init(); // Initialize NVS
    if (err==ESP_ERR_NVS_NO_FREE_PAGES || err==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); //NVS partition truncated and must be erased
        err = nvs_flash_init(); //Retry nvs_flash_init
    } ESP_ERROR_CHECK( err );

    //TODO: if no wifi setting found, trigger otamain
    
    //block that gets you WIFI with the lowest amount of effort, and based on FLASH
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    esp_netif_config.route_prio = 128;
    esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
    esp_wifi_set_default_wifi_sta_handlers();
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    //end of boilerplate code

    xTaskCreate(main_task,"main",4096,NULL,1,NULL);
    while (true) {
        vTaskDelay(1000); 
    }
    printf("app_main-done\n"); //will never exit here
}
