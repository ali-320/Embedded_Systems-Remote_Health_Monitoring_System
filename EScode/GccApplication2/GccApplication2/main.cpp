/*
 * GccApplication2.cpp
 *
 * Created: 12/23/2025 9:55:50 AM
 * Author : HOME
 */ 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ESP32 IDF Headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

// Pin Definitions
#define LO_PLUS_PIN    GPIO_NUM_10   // LO+ lead detection pin
#define LO_MINUS_PIN   GPIO_NUM_11   // LO- lead detection pin
#define ECG_ADC_PIN    ADC1_CHANNEL_0 // A0 analog input (GPIO36 on ESP32)

// UART Configuration
#define UART_NUM       UART_NUM_0    // Use UART0 for serial output
#define UART_BAUD_RATE 9600
#define UART_TX_PIN    GPIO_NUM_1    // Default TX pin
#define UART_RX_PIN    GPIO_NUM_3    // Default RX pin

// ADC Configuration
#define ADC_MAX_VALUE  4095          // 12-bit ADC
#define V_REF          1100          // Reference voltage in mV

// Logging tag
static const char *TAG = "ECG_READER";

// ADC calibration handle
static esp_adc_cal_characteristics_t *adc_chars = NULL;

// ---------- ADC Initialization ----------
void adc_init(void) {
	// Configure ADC1 channel
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ECG_ADC_PIN, ADC_ATTEN_DB_11);
	
	// Allocate memory for ADC calibration
	adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	
	// Characterize ADC for calibration
	esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc_chars);
	
	ESP_LOGI(TAG, "ADC initialized on GPIO36 (A0)");
}

// ---------- GPIO Initialization for Lead Detection ----------
void gpio_init_leads(void) {
	gpio_config_t io_conf = {
		.pin_bit_mask = (1ULL << LO_PLUS_PIN) | (1ULL << LO_MINUS_PIN),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	
	gpio_config(&io_conf);
	ESP_LOGI(TAG, "GPIO lead detection initialized (LO+: GPIO10, LO-: GPIO11)");
}

// ---------- UART Initialization for Serial Output ----------
void uart_init(void) {
	uart_config_t uart_config = {
		.baud_rate = UART_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};
	
	uart_driver_install(UART_NUM, 256, 0, 0, NULL, 0);
	uart_param_config(UART_NUM, &uart_config);
	uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	
	ESP_LOGI(TAG, "UART initialized at %d baud", UART_BAUD_RATE);
}

// ---------- Lead Detection Check ----------
int check_leads_off(void) {
	int lo_plus = gpio_get_level(LO_PLUS_PIN);
	int lo_minus = gpio_get_level(LO_MINUS_PIN);
	
	return (lo_plus == 1) || (lo_minus == 1);
}

// ---------- ECG Reading Function ----------
int read_ecg_value(void) {
	// Read raw ADC value
	uint32_t adc_reading = 0;
	
	// Average multiple readings for better accuracy
	for (int i = 0; i < 64; i++) {
		adc_reading += adc1_get_raw(ECG_ADC_PIN);
	}
	adc_reading /= 64;
	
	// Convert to voltage in mV (optional)
	// uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
	
	// For now, return the raw ADC value (0-4095)
	return (int)adc_reading;
}

// ---------- Main ECG Reading Task ----------
void ecg_read_task(void *pvParameters) {
	ESP_LOGI(TAG, "ECG Read Task Started");
	
	while (1) {
		// Check if leads are disconnected
		if (check_leads_off()) {
			uart_write_bytes(UART_NUM, "Leads Off!\n", strlen("Leads Off!\n"));
			ESP_LOGW(TAG, "Leads Off detected!");
			} else {
			// Read ECG value from ADC
			int ecgValue = read_ecg_value();
			
			// Send ECG value via UART
			char buffer[32];
			int len = snprintf(buffer, sizeof(buffer), "%d\n", ecgValue);
			uart_write_bytes(UART_NUM, (const char *)buffer, len);
		}
		
		// Delay 5 milliseconds (matching original Arduino code: delay(5))
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}

// ---------- Main Application Entry Point ----------
void app_main(void) {
	ESP_LOGI(TAG, "ECG Reader Application Started");
	
	// Initialize peripherals
	gpio_init_leads();
	adc_init();
	uart_init();
	
	ESP_LOGI(TAG, "All peripherals initialized. Starting ECG reading...");
	
	// Create ECG reading task with high priority
	xTaskCreate(ecg_read_task, "ecg_read_task", 2048, NULL, 10, NULL);
}
