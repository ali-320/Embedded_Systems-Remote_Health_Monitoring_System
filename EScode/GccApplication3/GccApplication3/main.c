/*
 * GccApplication3.c
 *
 * Created: 12/23/2025 10:19:45 AM
 * Author : HOME
 */ 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

// ESP32 IDF Headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_log.h"

// WiFi Configuration
#define WIFI_SSID      "EMBEDDED"
#define WIFI_PASSWORD  "12345678"
#define MAX_RETRIES    5

// UART Configuration
#define UART_NUM       UART_NUM_2
#define RX_PIN         GPIO_NUM_16
#define TX_PIN         GPIO_NUM_17
#define UART_BAUD_RATE 9600
#define UART_BUF_SIZE  (1024 * 2)

// LED Pin
#define LED_PIN        GPIO_NUM_2

// ECG Processing Parameters
#define REFRACTORY_PERIOD  100    // Time (ms) to ignore signal after a beat
#define MOVING_AVG_SIZE    3       // Smaller filter for faster response
#define RR_BUFFER_SIZE     10      // Buffer to store RR intervals for HRV calc

// Logging tag
static const char *TAG = "ECG_MONITOR";

// ---------- ECG Processing Variables ----------
volatile int ecgValue = 0;
volatile int currentBPM = 0;
volatile int currentRR = 0;
volatile int currentHRV = 0;  // rMSSD

// Signal tracking
int ecgBuffer[MOVING_AVG_SIZE] = {0};
int bufferIndex = 0;
int lastPeakValue = 0;
int peakCount = 0;

// Adaptive thresholding
int signalMin = 1024;
int signalMax = 0;
int baselineThreshold = 512;

// Timing and RR interval tracking
uint32_t lastBeatTime = 0;
uint32_t lastLogTime = 0;
int rrBuffer[RR_BUFFER_SIZE] = {0};
int rrIndex = 0;
int validRRCount = 0;
int sampleCount = 0;

// HTTP Server handle
httpd_handle_t server = NULL;

// ---------- Moving Average Filter ----------
int getFilteredECG(int rawValue) {
	ecgBuffer[bufferIndex] = rawValue;
	bufferIndex = (bufferIndex + 1) % MOVING_AVG_SIZE;
	
	int sum = 0;
	for (int i = 0; i < MOVING_AVG_SIZE; i++) {
		sum += ecgBuffer[i];
	}
	return sum / MOVING_AVG_SIZE;
}

// ---------- Adaptive Threshold Calculation ----------
void updateSignalStats(int value) {
	if (value > signalMax) signalMax = value;
	if (value < signalMin) signalMin = value;
	
	// Update baseline threshold as midpoint of min/max
	baselineThreshold = (signalMin + signalMax) / 2;
}

// ---------- Heart Rate Logic - Simplified Peak Detection ----------
void calculateHeartRate(int rawValue) {
	sampleCount++;
	
	// Update signal statistics for adaptive thresholding
	updateSignalStats(rawValue);
	
	// Apply moving average filter
	int filteredValue = getFilteredECG(rawValue);
	
	uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;  // Get time in milliseconds
	uint32_t timeSinceLastBeat = now - lastBeatTime;
	
	// Simple peak detection: detect when signal crosses threshold while ascending
	int crossingThreshold = (lastPeakValue < baselineThreshold && filteredValue >= baselineThreshold);
	
	// Only detect a new peak if minimum time has passed (refractory period)
	if (crossingThreshold && timeSinceLastBeat >= REFRACTORY_PERIOD) {
		peakCount++;
		
		// Calculate RR Interval directly from time
		int newRR = (int)timeSinceLastBeat;
		
		// ACCEPT ALL RR VALUES - no filtering on RR interval
		currentRR = newRR;
		lastBeatTime = now;
		
		// Calculate BPM from RR interval
		// BPM = 60000 / RR(ms)
		if (currentRR > 0) {
			currentBPM = 60000 / currentRR;
		}
		
		// Store RR value in buffer for HRV calculation
		rrBuffer[rrIndex] = currentRR;
		rrIndex = (rrIndex + 1) % RR_BUFFER_SIZE;
		
		// Track number of valid RR intervals (max 10)
		if (validRRCount < RR_BUFFER_SIZE) {
			validRRCount++;
		}
		
		// Calculate HRV (rMSSD) - needs at least 2 RR intervals
		if (validRRCount >= 2) {
			long sumSquaredDiffs = 0;
			int count = 0;
			
			// Calculate consecutive RR differences
			for (int i = 0; i < RR_BUFFER_SIZE - 1; i++) {
				int rr1 = rrBuffer[i];
				int rr2 = rrBuffer[(i + 1) % RR_BUFFER_SIZE];
				
				if (rr1 > 0 && rr2 > 0) {
					long diff = (long)rr1 - (long)rr2;
					sumSquaredDiffs += (diff * diff);
					count++;
				}
			}
			
			if (count > 0) {
				currentHRV = (int)sqrt((double)sumSquaredDiffs / (double)count);
			}
		}
		
		// Debug output every 1 second
		if (now - lastLogTime > 1000) {
			lastLogTime = now;
			ESP_LOGI(TAG, "Beat #%d | RR: %dms | BPM: %d | HRV: %d | Threshold: %d | Samples: %d | RRCount: %d",
			peakCount, currentRR, currentBPM, currentHRV, baselineThreshold, sampleCount, validRRCount);
		}
	}
	
	lastPeakValue = filteredValue;
}

// ---------- UART Read Task ----------
void uart_read_task(void *pvParameters) {
	uint8_t data[256];
	char inputString[64] = {0};
	int inputIndex = 0;
	
	ESP_LOGI(TAG, "UART Read Task Started");
	
	while (1) {
		int len = uart_read_bytes(UART_NUM, data, (sizeof(data) - 1), 20 / portTICK_PERIOD_MS);
		
		if (len > 0) {
			for (int i = 0; i < len; i++) {
				char c = (char)data[i];
				
				if (c == '\n' || c == '\r') {
					if (inputIndex > 0) {
						inputString[inputIndex] = '\0';
						int val = atoi(inputString);
						ecgValue = val;
						
						// Calculate heart rate metrics
						calculateHeartRate(val);
					}
					inputIndex = 0;
					memset(inputString, 0, sizeof(inputString));
					} else if (inputIndex < sizeof(inputString) - 1) {
					inputString[inputIndex++] = c;
				}
			}
		}
		
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

// ---------- Debug Task ----------
void debug_task(void *pvParameters) {
	static unsigned long lastDebugTime = 0;
	static int readCount = 0;
	
	while (1) {
		uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
		
		if (now - lastDebugTime > 2000) {
			ESP_LOGI(TAG, "[DEBUG] ECG samples processed | Peak detection threshold: %d | Total peaks: %d",
			baselineThreshold, peakCount);
			lastDebugTime = now;
			readCount = 0;
		}
		
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

// ---------- HTTP Handlers ----------
// Handler for /data endpoint (JSON response with ECG metrics)
esp_err_t data_handler(httpd_req_t *req) {
	char response[256];
	
	// Create JSON response
	int len = snprintf(response, sizeof(response),
	"{\"signal\":%d,\"bpm\":%d,\"rr\":%d,\"hrv\":%d}",
	ecgValue, currentBPM, currentRR, currentHRV);
	
	// Log every 10 requests to avoid spam
	static int logCounter = 0;
	logCounter++;
	if (logCounter >= 10) {
		ESP_LOGI(TAG, "[WEB] Sending: %s", response);
		logCounter = 0;
	}
	
	httpd_resp_set_header(req, "Cache-Control", "no-cache");
	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, response, len);
	
	return ESP_OK;
}

// Handler for / endpoint (HTML web page)
esp_err_t root_handler(httpd_req_t *req) {
	const char *html_response = "<!DOCTYPE html>"
	"<html>"
	"<head>"
	"<title>Digital Nurse ECG</title>"
	"<script src=\"https://cdn.jsdelivr.net/npm/chart.js\"></script>"
	"<style>"
	"body { font-family: 'Arial', sans-serif; background-color: #f4f7f6; text-align: center; margin: 0; padding: 20px; }"
	".dashboard { display: flex; justify-content: center; gap: 20px; margin-bottom: 20px; flex-wrap: wrap; }"
	".card { background: white; padding: 20px; border-radius: 12px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); width: 200px; transition: transform 0.2s; }"
	".card:hover { transform: translateY(-5px); }"
	".label { font-size: 14px; color: #7f8c8d; text-transform: uppercase; letter-spacing: 1px; }"
	".value { font-size: 36px; font-weight: bold; margin: 10px 0; color: #2c3e50; }"
	".unit { font-size: 12px; color: #95a5a6; }"
	".normal { color: #27ae60; }"
	".warning { color: #f39c12; }"
	".critical { color: #c0392b; }"
	"canvas { background: white; border-radius: 12px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); max-width: 1000px; margin: 0 auto; }"
	"</style>"
	"</head>"
	"<body>"
	"<h2>Live ECG Monitor</h2>"
	"<div class=\"dashboard\">"
	"<div class=\"card\">"
	"<div class=\"label\">Heart Rate</div>"
	"<div class=\"value\" id=\"bpmValue\">--</div>"
	"<div class=\"unit\">BPM</div>"
	"</div>"
	"<div class=\"card\">"
	"<div class=\"label\">RR Interval</div>"
	"<div class=\"value\" id=\"rrValue\">--</div>"
	"<div class=\"unit\">Milliseconds</div>"
	"</div>"
	"<div class=\"card\">"
	"<div class=\"label\">HRV (Stress)</div>"
	"<div class=\"value\" id=\"hrvValue\">--</div>"
	"<div class=\"unit\">rMSSD</div>"
	"</div>"
	"</div>"
	"<canvas id=\"ecgChart\" width=\"1000\" height=\"400\"></canvas>"
	"<script>"
	"const ctx = document.getElementById('ecgChart').getContext('2d');"
	"const data = { labels: [], datasets: [{ label: 'ECG Signal', borderColor: '#e74c3c', borderWidth: 2, data: [], fill: false, pointRadius: 0, tension: 0.4 }] };"
	"const config = { type: 'line', data: data, options: { animation: false, responsive: true, scales: { x: { display: false }, y: { min: 0, max: 1024 } } } };"
	"const ecgChart = new Chart(ctx, config);"
	"async function updateData() {"
		"try {"
			"const response = await fetch('/data');"
			"const jsonData = await response.json();"
			"if (data.labels.length > 200) { data.labels.shift(); data.datasets[0].data.shift(); }"
			"data.labels.push('');"
			"data.datasets[0].data.push(jsonData.signal);"
			"ecgChart.update();"
			"updateBPM(jsonData.bpm);"
			"document.getElementById('rrValue').innerText = jsonData.rr > 0 ? jsonData.rr + ' ms' : '--';"
			"document.getElementById('hrvValue').innerText = jsonData.hrv > 0 ? jsonData.hrv : '--';"
			"} catch(err) { console.log('Error:', err); }"
		"}"
		"function updateBPM(bpm) {"
			"const bpmEl = document.getElementById('bpmValue');"
			"bpmEl.innerText = bpm > 0 ? bpm : '--';"
			"bpmEl.classList.remove('normal', 'warning', 'critical');"
			"if (bpm >= 60 && bpm <= 100) { bpmEl.classList.add('normal'); }"
			"else if (bpm > 0 && bpm < 60 && bpm > 40) { bpmEl.classList.add('warning'); }"
			"else if (bpm > 100 && bpm < 120) { bpmEl.classList.add('warning'); }"
			"else if (bpm > 0) { bpmEl.classList.add('critical'); }"
		"}"
		"setInterval(updateData, 50);"
		"</script>"
		"</body>"
		"</html>";
		
		httpd_resp_set_header(req, "Cache-Control", "no-cache");
		httpd_resp_set_type(req, "text/html");
		httpd_resp_send(req, html_response, strlen(html_response));
		
		return ESP_OK;
	}

	// ---------- HTTP Server Configuration ----------
	httpd_uri_t uri_get_data = {
		.uri       = "/data",
		.method    = HTTP_GET,
		.handler   = data_handler,
		.user_ctx  = NULL
	};

	httpd_uri_t uri_get_root = {
		.uri       = "/",
		.method    = HTTP_GET,
		.handler   = root_handler,
		.user_ctx  = NULL
	};

	void start_webserver(void) {
		httpd_config_t config = HTTPD_DEFAULT_CONFIG();
		config.server_port = 80;
		
		if (httpd_start(&server, &config) == ESP_OK) {
			httpd_register_uri_handler(server, &uri_get_data);
			httpd_register_uri_handler(server, &uri_get_root);
			ESP_LOGI(TAG, "Web server started on port 80");
			} else {
			ESP_LOGE(TAG, "Failed to start web server");
		}
	}

	void stop_webserver(void) {
		if (server) {
			httpd_stop(server);
			ESP_LOGI(TAG, "Web server stopped");
		}
	}

	// ---------- WiFi Event Handler ----------
	void event_handler(void *arg, esp_event_base_t event_base,
	int32_t event_id, void *event_data) {
		if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
			esp_wifi_connect();
			ESP_LOGI(TAG, "WiFi connect requested");
			} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
			esp_wifi_connect();
			ESP_LOGI(TAG, "WiFi disconnected, attempting reconnect");
			} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
			ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
			ESP_LOGI(TAG, "WiFi connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
			start_webserver();
		}
	}

	void wifi_init_sta(void) {
		esp_netif_create_default_wifi_sta();
		
		wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
		esp_wifi_init(&cfg);
		
		esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL);
		esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL);
		
		wifi_config_t wifi_config = {
			.sta = {
				.ssid = WIFI_SSID,
				.password = WIFI_PASSWORD,
			},
		};
		
		esp_wifi_set_mode(WIFI_MODE_STA);
		esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
		esp_wifi_start();
		
		ESP_LOGI(TAG, "WiFi initialization finished. Connecting to %s...", WIFI_SSID);
	}

	// ---------- UART Initialization ----------
	void uart_init(void) {
		uart_config_t uart_config = {
			.baud_rate = UART_BAUD_RATE,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_APB,
		};
		
		uart_driver_install(UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);
		uart_param_config(UART_NUM, &uart_config);
		uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
		
		ESP_LOGI(TAG, "UART initialized on UART_NUM_2 (RX: GPIO16)");
	}

	// ---------- GPIO Initialization ----------
	void gpio_init(void) {
		gpio_config_t io_conf = {
			.pin_bit_mask = (1ULL << LED_PIN),
			.mode = GPIO_MODE_OUTPUT,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE,
		};
		gpio_config(&io_conf);
		ESP_LOGI(TAG, "GPIO initialized (LED on GPIO 2)");
	}

	// ---------- Main Application Entry Point ----------
	void app_main(void) {
		// Initialize NVS
		esp_err_t ret = nvs_flash_init();
		if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
			ESP_ERROR_CHECK(nvs_flash_erase());
			ret = nvs_flash_init();
		}
		ESP_ERROR_CHECK(ret);
		
		// Initialize event loop
		ESP_ERROR_CHECK(esp_event_loop_create_default());
		
		// Initialize GPIO and UART
		gpio_init();
		uart_init();
		
		// Initialize WiFi
		ESP_LOGI(TAG, "Initializing WiFi...");
		wifi_init_sta();
		
		// Create UART read task
		xTaskCreate(uart_read_task, "uart_read_task", 4096, NULL, 10, NULL);
		
		// Create debug task
		xTaskCreate(debug_task, "debug_task", 2048, NULL, 5, NULL);
		
		ESP_LOGI(TAG, "ECG Monitor application started");
	}
