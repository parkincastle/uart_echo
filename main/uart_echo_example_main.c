/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "esp_timer.h"
#include "esp_event.h"
#include "esp_system.h"
#include "freertos/event_groups.h"
#include "driver/timer.h"

#include "mqtt_client.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs_flash.h"

#include <sys/time.h>
#include "esp_attr.h"
#include "esp_sleep.h"

/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */
#define SENSOR_UART_NUM UART_NUM_2
#define COMM_UART_NUM UART_NUM_1

#define soundSensor_id 3

#define SENSOR_TX_PIN GPIO_NUM_1
#define SENSOR_RX_PIN GPIO_NUM_2
#define SENSOR_RTS UART_PIN_NO_CHANGE
#define SENSOR_CTS UART_PIN_NO_CHANGE

// #define COMM_TX_PIN GPIO_NUM_36
// #define COMM_RX_PIN GPIO_NUM_45

#define COMM_TX_PIN GPIO_NUM_12
#define COMM_RX_PIN GPIO_NUM_11
#define COMM_RTS UART_PIN_NO_CHANGE
#define COMM_CTS UART_PIN_NO_CHANGE

#define ECHO_TASK_STACK_SIZE    (4096)

#define EXAMPLE_READ_LEN   100
#define EXAMPLE_ADC_CONV_MODE           ADC_CONV_SINGLE_UNIT_1
#define TIMER_DIVIDER         16
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // used to calculate counter value 
#define TIMER_INTERVAL_SEC    (5)                              // test interval for timer (seconds)


#define PATTERN_CHR_NUM    (3) 

static const char *TAG = "Aging_Test";

#define BUF_SIZE (256)

SemaphoreHandle_t xSemaphore;

uint32_t binary3_to_dec(uint8_t byte1, uint8_t byte2, uint8_t byte3){
    uint32_t hex_value = 0;
    hex_value = (byte1 << 16) | (byte2 << 8) | byte3;
    return hex_value;
}
uint32_t binary2_to_dec(uint8_t byte1, uint8_t byte2){
    uint32_t hex_value = 0;
    hex_value = (byte1 << 8) | byte2;
    return hex_value;
}

static void uart_init(){
    // uart 초기세팅
    int intr_alloc_flags = 0;
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */

    uart_config_t comm_uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    // wi-sun uart setting
    ESP_ERROR_CHECK(uart_param_config(COMM_UART_NUM, &comm_uart_config));
    ESP_ERROR_CHECK(uart_set_pin(COMM_UART_NUM, COMM_TX_PIN, COMM_RX_PIN, COMM_RTS, COMM_CTS));
    ESP_ERROR_CHECK(uart_driver_install(COMM_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
}

static bool check_crc(int data_len, uint32_t actual_crc, uint8_t *data){
    int cal_crc = 0xFFFF;
    // CRC계산 
    for (int i = 0; i < data_len - 2; i++){
        cal_crc ^= data[i];
        for (int j = 0; j < 8; j++){
            if (cal_crc & 0x0001){
                cal_crc >>= 1;
                cal_crc ^= 0xA001; // Modbus CRC-16 계산
            }else{
                cal_crc >>= 1;
            }
        }
    }
    // 실제 crc와 계산된 crc비교
    if (cal_crc != actual_crc){
        ESP_LOGI(TAG, "ERROR : CRC value deosn't match restart loop.\n CALC_CRC value : %02X CRC : %02lX", cal_crc, actual_crc);
        // for (int i = 0; i < data_len; i++){
        //     printf("%02X ", *(char *)&data[i]);
        // }
        // printf("\n\n\r");
        return false;
    }
    return true;
}

void setting_task(void *pvParameters){
    char boder_setting[] =   "\rmacf deny\rmacf allow 001d129f35c51b94\rmode 1\rchan 33 59\rip 2001:db8::1/64\rleaseip 001d129f35c51b94 2001:db8::2/64\ratstart 1\rtcpopts send_done on\rudpopts send_done on\rsave\rreset";
    char router_setting[] =  "\rmacf deny\rmacf allow 001d129f35c526c0\rmacf allow 001d129f35c51b94\rmode 1\rchan 33 59\ratstart 2\rtcpopts send_done on\rudpopts send_done on\rsave\rreset";
    char leaf_setting[] =    "\rmacf deny\rmacf allow 001d129f35c526c0\rmacf allow 001d129f35c51b94\rmode 1\rchan 33 59\ratstart 3\rtcpopts send_done on\rudpopts send_done on\rsave\rreset";
    char *resetCommand =     "\rclear\rreset\r";

    uint8_t *setting_data = (uint8_t *)malloc(BUF_SIZE * 2);
    int setting_len = 0;
    int cnt = 0;

    uart_flush_input(COMM_UART_NUM);
    
    while(1) {
        if (xSemaphoreTake(xSemaphore, (TickType_t)100) == pdTRUE){
            if(cnt < 2){
                uart_write_bytes(COMM_UART_NUM, resetCommand, strlen(resetCommand));
                uart_wait_tx_done(COMM_UART_NUM, pdMS_TO_TICKS(200));
                uart_flush_input(COMM_UART_NUM);
                vTaskDelay(300);
                uart_read_bytes(COMM_UART_NUM, setting_data, 200, 10 * portTICK_PERIOD_MS);
                uart_write_bytes(COMM_UART_NUM, leaf_setting, strlen(leaf_setting));
                uart_wait_tx_done(COMM_UART_NUM, pdMS_TO_TICKS(500));
                cnt ++;
            }
            setting_len = uart_read_bytes(COMM_UART_NUM, setting_data, 512, 10 * portTICK_PERIOD_MS);

            ESP_LOGI("setting_task", "wi-sun setting read byte : %d, Count : %d\n", setting_len, cnt);
            xSemaphoreGive(xSemaphore);
        }
        
        if(setting_len){
            ESP_LOGI("setting_task" , "wi-sun setting receive data : ");
            for(int i = 0; i < setting_len; i++){
                printf("%02X", setting_data[i]);
            }
            printf("\r\n");

            char* ascii_result = hex_to_ascii(setting_data, setting_len);
            printf("ASCII : %s\n", ascii_result);

            free(ascii_result);
            setting_len = 0;
        }
        vTaskDelay(400);
    }
    free(setting_data);
}



void app_main(void){
    // udps 2001:db8::1 3610 {센서값들 ...}
    char wiSun_command[1060];

    char WiSUN_ack_data_l[] = "tcpsd <2001:db8::1>";
    char WiSUN_ack_data_s[] = "tcpsd";

    uint8_t *wiSun_data = (uint8_t *)malloc(BUF_SIZE);
    char *hex_string = (char *)malloc(BUF_SIZE);

    // char ascii_result[256] = {0x00};
    int board_len = 0;
    int tx_done_check = 1;
    int loop_cnt = 0;
    int recv_cnt = 0;
    int gpio = 0;

    xSemaphore = xSemaphoreCreateMutex();
    xTaskCreate(setting_task, "setting_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);


    // GPIO9을 출력 모드로 설정(wi-sun보드 전원)
    gpio_config_t wisun_gpio = {
        .pin_bit_mask = (1ULL<<GPIO_NUM_9),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&wisun_gpio);
    gpio = gpio_set_level(GPIO_NUM_9, 1);

    uart_init();

    ESP_LOGI(TAG, "delay 3min 20sec for wi-sun linking");
    vTaskDelay(20000);

    ESP_LOGI(TAG, "Start app_main");

    sprintf(wiSun_command, "tcps 2001:db8::3 3610 4605001621022280277895732951750538578104397321929067512058414600568584935159923732487836507329993993292714466920980130379745257915727861586905028741672591356874570796150727475390741550892162699400004659080882276943639837178616417796519041111355013789308672424096887455864058897511537110108758557448393462372261086016093219742051559301622929801352562222271261665321076672177804948618383978871661115961658746852354580899999354366669813816720341142030364208240486459271714386114728991171159742871761771730300782003342701493363069521040049962634944410088705357403824796752946805061896605399227732490745368241600510832880468149291022028287361680410711414566887642123433031260456110052164510006978782887428386832183407950485246117225445690104394747383896921711311668402381730156654310783568009928799996411179867201001817272097562469291848713497365545328705786076614662817351185877421856891234934063812394940095821440145806108739621724553633216929777254111972948398632617537473075119729812709670830656640369818166020980076188550844\r");

    while (1){       
        loop_cnt++;
        ESP_LOGI(TAG, "loop_cnt : %d", loop_cnt);

        uart_flush_input(COMM_UART_NUM);

        if (xSemaphoreTake(xSemaphore, (TickType_t)100) == pdTRUE){
            // wi-sun통신 모듈로 커맨드 발송
            uart_write_bytes(COMM_UART_NUM, wiSun_command, strlen(wiSun_command));

            // tx가 끝날때까지 최대 1초 동안 대기
            tx_done_check = uart_wait_tx_done(COMM_UART_NUM, pdMS_TO_TICKS(1000));

            // rx buf 초기화
            uart_flush_input(COMM_UART_NUM);

            // 수신 완료 커맨드(tcpsd <2001:db8::1> == 7463707364203C323030313A6462383A3A313E) 읽어옴
            board_len = uart_read_bytes(COMM_UART_NUM, wiSun_data, 100, 10 * portTICK_PERIOD_MS);
            xSemaphoreGive(xSemaphore);
        }

        if(tx_done_check == 0){
            ESP_LOGI(TAG, "UART TX done!");
        }

        ESP_LOGI(TAG, "wi-sun read byte : %d", board_len);

        char *char_data = (char *)wiSun_data;

        if(board_len){
            // // "tcpsd <2001:db8::1>" 전체 ack값을 판별해 정확하다
            // char* ptr = strstr(char로 바꾼 wi-sun hex string, WiSUN_ack_data_l);
            // "tcpsd" 만 판별해 범용성이 좋음
            char* ptr = strstr(char_data, WiSUN_ack_data_s);

            if(ptr != NULL){
                recv_cnt++;
                ESP_LOGI(TAG, "recv_cnt : %d", recv_cnt);
            }

            printf("wi-sun receive data : ");
            for(int i = 0; i < board_len; i++){
                printf("%02X", wiSun_data[i]);
            }
            printf("\n");

            printf("wi-sun char receive data : \n%s\n\n", char_data);

            // hex_to_ascii(wiSun_data, ascii_result);
            // printf("ASCII : %s\n", ascii_result);
        }else{
            ESP_LOGI("aging_test", "wi-sun doesn't receive.");
        }

        if(loop_cnt > 999){
            printf("\n\n");
            ESP_LOGI(TAG, "aging test End \n senf : %d, receive : %d, Loss : %f", loop_cnt, recv_cnt, ((float)recv_cnt / (float)loop_cnt) * 100  );
            break;
        }

        tx_done_check = 1;
        memset(wiSun_data, 0, BUF_SIZE);
        vTaskDelay(100);
    }
    free(wiSun_data);
}