//Standard Libraries
#include <stdio.h>
#include <string.h>

//Board specific libraries
#include "sdkconfig.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_spi_flash.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <u8g2.h>

//Display library
#include "u8g2_esp32_hal.h"

#define BLINK_GPIO 25
#define PIN_SDA 4
#define PIN_SCL 15

static const char *TAG = "ssd1306";

void app_main(void){
	gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
	


    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda   = PIN_SDA;
	u8g2_esp32_hal.scl  = PIN_SCL;
	u8g2_esp32_hal_init(u8g2_esp32_hal);

	u8g2_t u8g2; // a structure which will contain all the data for one display
	u8g2_Setup_ssd1306_i2c_128x64_noname_f( //U8G2_SSD1306_128X64_NONAME_F_SW_I2C
		&u8g2,
		U8G2_R0,
		//u8x8_byte_sw_i2c,
		u8g2_esp32_i2c_byte_cb,
		u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
	u8x8_SetI2CAddress(&u8g2.u8x8,0x3C);

	ESP_LOGI(TAG, "u8g2_InitDisplay");
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

	gpio_set_level(BLINK_GPIO, 1);	

	ESP_LOGI(TAG, "u8g2_SetPowerSave");
	u8g2_SetPowerSave(&u8g2, 0); // wake up display

	ESP_LOGI(TAG, "u8g2_ClearBuffer");
	u8g2_ClearBuffer(&u8g2);
	ESP_LOGI(TAG, "u8g2_DrawBox");
	u8g2_DrawBox(&u8g2, 0, 26, 80,6);
	u8g2_DrawFrame(&u8g2, 0,26,100,6);

	ESP_LOGI(TAG, "u8g2_SetFont");
    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
	ESP_LOGI(TAG, "u8g2_DrawStr");
    u8g2_DrawStr(&u8g2, 2,17,"Hi nkolban!");
	ESP_LOGI(TAG, "u8g2_SendBuffer");
	u8g2_SendBuffer(&u8g2);

	ESP_LOGI(TAG, "All done!");

	vTaskDelete(NULL);
	
}
