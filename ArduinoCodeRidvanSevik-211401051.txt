// --- LIBRARIES ---
#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/i2s.h"      // ESP-IDF I2S Driver
#include <Adafruit_NeoPixel.h> // Include Adafruit NeoPixel library

#define BLYNK_PRINT Serial     // Redirect debug output for Blynk to Serial Port
#include "secrets.h"           // File containing secret information (WiFi passwords, Blynk token, Telegram token, etc.)
#include <BlynkSimpleEsp32.h>
#include <HTTPClient.h>

// --- Debug Level ---
#define DEBUG_LEVEL 2 // You can set it to 0, 1, or 2 according to your needs
#if DEBUG_LEVEL > 0
#define DEBUG_PRINT(level, format, ...) if (DEBUG_LEVEL >= level) { Serial.printf(PSTR("[%s] " format), millisToTimeStr(), ##__VA_ARGS__); }
#else
#define DEBUG_PRINT(level, format, ...)
#endif

// Helper function for timestamp
const char* millisToTimeStr() {
    static char time_str[15];
    unsigned long ms = millis();
    unsigned long s = ms / 1000;
    unsigned long m = s / 60;
    unsigned long h = m / 60;
    ms %= 1000;
    s %= 60;
    m %= 60;
    sprintf(time_str, "%02lu:%02lu:%02lu.%03lu", h, m, s, ms);
    return time_str;
}

// --- Wi-Fi Information (will come from secrets.h file as WIFI_SSID and WIFI_PASSWORD) ---
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const int WIFI_CONNECT_TIMEOUT_MS = 20000; // 20 seconds

// --- Blynk Settings (will come from secrets.h file as BLYNK_AUTH_TOKEN) ---
char blynk_auth[] = BLYNK_AUTH_TOKEN;

// --- Telegram Settings (will come from secrets.h file as BOT_TOKEN and CHAT_ID) ---
String botToken = BOT_TOKEN;
String chatID = CHAT_ID;

// --- UDP Settings ---
const char* udpAddress = "192.168.149.17"; // YOUR PYTHON SERVER'S IP ADDRESS
const int udpTargetPort = 12345;          // Port where the Python server listens for AUDIO
const int udpLocalPort = 12345;           // Port where ESP32 will listen for AUDIO
const int udpGasTargetPort = 12346;       // NEW: Port where the Python server listens for GAS DATA
WiFiUDP udp;

// --- I2S General Settings ---
#define I2S_SAMPLE_RATE 16000
#define I2S_BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_16BIT

// --- I2S Pins and Settings for Microphone Input (I2S_NUM_0) ---
#define I2S_MIC_WS          15  // LRCK
#define I2S_MIC_SD          14  // DOUT (Data IN for ESP32)
#define I2S_MIC_SCK         13  // BCLK
#define I2S_MIC_CHANNEL_FMT I2S_CHANNEL_FMT_ONLY_LEFT // For mono microphone
#define I2S_MIC_DMA_BUF_COUNT 4
#define I2S_MIC_DMA_BUF_LEN   256
#define I2S_MIC_READ_LEN      1024 // Buffer size to be read from microphone at once (bytes)
char i2s_mic_read_buffer[I2S_MIC_READ_LEN];

// --- I2S Pins and Settings for Speaker Output (I2S_NUM_1) ---
#define I2S_SPEAKER_WS      10  // GPIO you connected to PCM5102A LRCK pin
#define I2S_SPEAKER_SD      12  // GPIO you connected to PCM5102A DIN pin (Data OUT for ESP32)
#define I2S_SPEAKER_SCK     8   // GPIO you connected to PCM5102A BCK pin
#define I2S_SPEAKER_CHANNEL_FMT I2S_CHANNEL_FMT_ONLY_LEFT // We assume mono audio comes from the server
#define I2S_SPEAKER_DMA_BUF_COUNT 10
#define I2S_SPEAKER_DMA_BUF_LEN   512
#define INCOMING_AUDIO_BUFFER_SIZE 2048 // Buffer size for incoming audio
char incomingAudioBuffer[INCOMING_AUDIO_BUFFER_SIZE];

// --- Status LED (NeoPixel Settings) ---
#define NEOPIXEL_PIN 48      // Internal RGB LED pin for ESP32-S3-DevKitM-1U
#define NEOPIXEL_COUNT 1     // Usually, the internal LED is a single unit
Adafruit_NeoPixel statusPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
uint32_t color_off, color_wifi_connecting, color_wifi_connected, color_audio_playing, color_mic_active, color_gas_alert;

// --- Audio Playback Status and Timeout ---
bool isPlayingAudio = false;
unsigned long lastAudioPacketTime = 0;
const unsigned long audioTimeoutMs = 500;      // Max waiting time between audio packets
const unsigned long i2sWriteTimeoutMs = 100;   // Max waiting time for I2S write operation

// --- MQ2 Gas Sensor and Buzzer Pins ---
#define MQ2_PIN 4      // Must be a pin capable of analog reading (e.g., GPIO4, ADC1_CH3)
#define BUZZER_PIN 18  // Buzzer pin. Make sure it does not conflict with I2S pins.

int threshold = 1000;          // Initial threshold value (can be changed via V1 on Blynk)
bool gas_alert_active = false; // Track whether the gas alarm is active

// --- Timing Variables ---
unsigned long lastMq2Check = 0;
const unsigned long mq2CheckInterval = 750; // MQ2 check interval in ms

// NEW: Timing for sending gas data
unsigned long lastGasSendTime = 0;
const unsigned long gasSendInterval = 2000; // Gas data sending interval in ms (2 seconds)

// --- I2S Microphone Setup ---
void setupI2SMic() {
    DEBUG_PRINT(1, "I2S_MIC: Setting up I2S Microphone (I2S_NUM_0)...\n");
    const i2s_config_t i2s_mic_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE,
        .channel_format = I2S_MIC_CHANNEL_FMT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = I2S_MIC_DMA_BUF_COUNT,
        .dma_buf_len = I2S_MIC_DMA_BUF_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    esp_err_t err_install = i2s_driver_install(I2S_NUM_0, &i2s_mic_config, 0, NULL);
    if (err_install != ESP_OK) {
        Serial.printf(PSTR("[%s] I2S_MIC: ERROR: Failed to install I2S_NUM_0 driver! Error Code: %d (%s)\n"), millisToTimeStr(), err_install, esp_err_to_name(err_install));
        return;
    }
    DEBUG_PRINT(1, "I2S_MIC: I2S_NUM_0 driver installed successfully.\n");
    const i2s_pin_config_t mic_pin_config = {
        .bck_io_num = I2S_MIC_SCK,
        .ws_io_num = I2S_MIC_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_MIC_SD
    };
    esp_err_t err_pin = i2s_set_pin(I2S_NUM_0, &mic_pin_config);
    if (err_pin != ESP_OK) {
        Serial.printf(PSTR("[%s] I2S_MIC: ERROR: Failed to set I2S_NUM_0 pins! Error Code: %d (%s)\n"), millisToTimeStr(), err_pin, esp_err_to_name(err_pin));
    } else {
        DEBUG_PRINT(1, "I2S_MIC: I2S_NUM_0 pins set successfully.\n");
    }
    DEBUG_PRINT(1, "I2S_MIC: I2S Microphone (I2S_NUM_0) setup completed.\n");
}

// --- I2S Speaker Setup ---
void setupI2SSpeaker() {
    DEBUG_PRINT(1, "I2S_SPK: Setting up I2S Speaker (I2S_NUM_1)...\n");
    const i2s_config_t i2s_speaker_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE,
        .channel_format = I2S_SPEAKER_CHANNEL_FMT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = I2S_SPEAKER_DMA_BUF_COUNT,
        .dma_buf_len = I2S_SPEAKER_DMA_BUF_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };
    esp_err_t err_install = i2s_driver_install(I2S_NUM_1, &i2s_speaker_config, 0, NULL);
    if (err_install != ESP_OK) {
        Serial.printf(PSTR("[%s] I2S_SPK: ERROR: Failed to install I2S_NUM_1 driver! Error Code: %d (%s)\n"), millisToTimeStr(), err_install, esp_err_to_name(err_install));
        return;
    }
    DEBUG_PRINT(1, "I2S_SPK: I2S_NUM_1 driver installed successfully.\n");
    const i2s_pin_config_t speaker_pin_config = {
        .bck_io_num = I2S_SPEAKER_SCK,
        .ws_io_num = I2S_SPEAKER_WS,
        .data_out_num = I2S_SPEAKER_SD,
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    esp_err_t err_pin = i2s_set_pin(I2S_NUM_1, &speaker_pin_config);
    if (err_pin != ESP_OK) {
        Serial.printf(PSTR("[%s] I2S_SPK: ERROR: Failed to set I2S_NUM_1 pins! Error Code: %d (%s)\n"), millisToTimeStr(), err_pin, esp_err_to_name(err_pin));
    } else {
        DEBUG_PRINT(1, "I2S_SPK: I2S_NUM_1 pins set successfully.\n");
    }
    DEBUG_PRINT(1, "I2S_SPK: I2S Speaker (I2S_NUM_1) setup completed.\n");
}

// --- Telegram Message Sending Function ---
void sendTelegramMessage(String message) {
    if (WiFi.status() != WL_CONNECTED) {
        DEBUG_PRINT(1, "TELEGRAM: WiFi not connected, message cannot be sent: %s\n", message.c_str());
        return;
    }
    DEBUG_PRINT(1, "TELEGRAM: Sending message: %s\n", message.c_str());
    HTTPClient http;
    // URL encode message for safety if it contains special characters
    String urlEncodedMessage = "";
    for (char c : message) {
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~' || c == ' ') {
            urlEncodedMessage += (c == ' ' ? "%20" : String(c));
        } else {
            char buff[4];
            sprintf(buff, "%%%02X", (unsigned char)c);
            urlEncodedMessage += buff;
        }
    }
    String url = "https://api.telegram.org/bot" + botToken + "/sendMessage?chat_id=" + chatID + "&text=" + urlEncodedMessage;
    
    DEBUG_PRINT(2, "TELEGRAM: URL: %s\n", url.c_str());
    http.begin(url); // HTTPS by default if URL starts with https://
    // For ESP32, if you face issues with HTTPS, you might need to provide a root CA certificate
    // http.begin(client, url); // where client is a WiFiClientSecure object

    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
        DEBUG_PRINT(2, "TELEGRAM: Message sent, HTTP Response Code: %d\n", httpResponseCode);
        String responsePayload = http.getString();
        DEBUG_PRINT(2, "TELEGRAM: Response Content: %s\n", responsePayload.c_str());
    } else {
        DEBUG_PRINT(1, "TELEGRAM: ERROR: Message could not be sent, HTTP Error Code: %d, Error: %s\n", httpResponseCode, http.errorToString(httpResponseCode).c_str());
    }
    http.end();
}

// --- Get Blynk Threshold Value (from V1 virtual pin) ---
BLYNK_WRITE(V1) {
    int newThreshold = param.asInt();
    DEBUG_PRINT(1, "BLYNK: New gas threshold value received from V1: %d\n", newThreshold);
    if (newThreshold > 0 && newThreshold < 4096) { // Check if it's in a reasonable range
        threshold = newThreshold;
        DEBUG_PRINT(1, "BLYNK: Gas threshold value updated to %d.\n", threshold);
    } else {
        DEBUG_PRINT(1, "BLYNK: Received threshold value (%d) is invalid, not updated.\n", newThreshold);
    }
}

// --- MAIN SETUP FUNCTION ---
void setup() {
    Serial.begin(115200);
    unsigned long setup_start_time = millis();
    for(int i=0; i<10 && !Serial; i++) delay(100);
    DEBUG_PRINT(1, "SETUP: ESP32 Initializing...\n");

    statusPixel.begin();
    statusPixel.setBrightness(20); 
    
    color_off               = statusPixel.Color(0, 0, 0);
    color_wifi_connecting   = statusPixel.Color(0, 0, 25);
    color_wifi_connected    = statusPixel.Color(0, 25, 0);
    color_audio_playing     = statusPixel.Color(25, 0, 25);
    color_mic_active        = statusPixel.Color(25, 25, 0);
    color_gas_alert         = statusPixel.Color(50, 0, 0);
    statusPixel.setPixelColor(0, color_off);
    statusPixel.show();

    DEBUG_PRINT(1, "SETUP: Setting MQ2 pin (GPIO%d) as INPUT, Buzzer pin (GPIO%d) as OUTPUT.\n", MQ2_PIN, BUZZER_PIN);
    pinMode(MQ2_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH); // Buzzer initially off (assuming HIGH = off)

    DEBUG_PRINT(1, "WIFI: Connecting to Wi-Fi Network: %s\n", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    unsigned long wifi_start_time = millis();
    bool wifi_led_blink_state = false;

    while (WiFi.status() != WL_CONNECTED) {
        wifi_led_blink_state = !wifi_led_blink_state;
        statusPixel.setPixelColor(0, wifi_led_blink_state ? color_wifi_connecting : color_off);
        statusPixel.show();
        delay(300);
        Serial.print(".");
        if (millis() - wifi_start_time > WIFI_CONNECT_TIMEOUT_MS) {
            Serial.printf(PSTR("\n[%s] WIFI: ERROR: Wi-Fi connection timed out! (%lu ms). Restarting...\n"), millisToTimeStr(), WIFI_CONNECT_TIMEOUT_MS);
            ESP.restart();
        }
    }
    Serial.printf(PSTR("\n[%s] WIFI: Wi-Fi Connected!\n"), millisToTimeStr());
    Serial.printf(PSTR("[%s] WIFI: IP Address: %s\n"), millisToTimeStr(), WiFi.localIP().toString().c_str());
    statusPixel.setPixelColor(0, color_wifi_connected);
    statusPixel.show();

    DEBUG_PRINT(1, "BLYNK: Connecting to Blynk (Token: %sxxxx)\n", String(blynk_auth).substring(0,4).c_str() ); // Don't log the entire token
    Blynk.config(blynk_auth); 
    
    unsigned long blynk_connect_start = millis();
    bool blynk_connected_successfully = false;
    while(millis() - blynk_connect_start < 10000){ 
        if (Blynk.connect(500)) { 
            DEBUG_PRINT(1, "BLYNK: Blynk connected successfully.\n");
            blynk_connected_successfully = true;
            break;
        }
        // DEBUG_PRINT(2, "BLYNK: Connection attempt ongoing...\n"); // Could be very frequent logging
    }
    if (!blynk_connected_successfully) {
        DEBUG_PRINT(1, "BLYNK: WARNING: Could not connect to Blynk within 10 seconds during setup. Will keep trying in loop.\n");
    }

    DEBUG_PRINT(1, "UDP: Starting UDP listener (for audio) on port %d...\n", udpLocalPort);
    if (udp.begin(udpLocalPort)) { // This is to listen for incoming AUDIO packets
        DEBUG_PRINT(1, "UDP: UDP listener started successfully on port %d.\n", udpLocalPort);
    } else {
        Serial.printf(PSTR("[%s] UDP: ERROR: Failed to start UDP listener!\n"), millisToTimeStr());
    }

    setupI2SMic();
    setupI2SSpeaker();

    DEBUG_PRINT(1, "I2S_SPK: Initially stopping I2S_NUM_1 (speaker) and clearing buffer.\n");
    esp_err_t err_zero = i2s_zero_dma_buffer(I2S_NUM_1);
    if (err_zero != ESP_OK) DEBUG_PRINT(1, "I2S_SPK: WARNING: Initial i2s_zero_dma_buffer error: %s\n", esp_err_to_name(err_zero));
    
    esp_err_t err_stop_init = i2s_stop(I2S_NUM_1);
    if (err_stop_init != ESP_OK) Serial.printf(PSTR("[%s] I2S_SPK: ERROR: I2S_NUM_1 could not be stopped initially! Error: %s\n"), millisToTimeStr(), esp_err_to_name(err_stop_init));
    else DEBUG_PRINT(1, "I2S_SPK: I2S_NUM_1 (speaker) stopped initially.\n");
    
    DEBUG_PRINT(1, "SETUP: Setup took %lu ms. Starting loop...\n", millis() - setup_start_time);
}


// --- MAIN LOOP FUNCTION ---
void loop() {
    unsigned long currentTime = millis();

    if (WiFi.status() == WL_CONNECTED) {
        if (!Blynk.connected()) {
            DEBUG_PRINT(1, "LOOP: Blynk connection lost! Attempting to reconnect...\n");
            // Blynk.connect() can be blocking. Blynk.run() already tries.
        }
        Blynk.run();
    } else {
        DEBUG_PRINT(1, "LOOP: WiFi connection lost! Attempting to reconnect...\n");
        statusPixel.setPixelColor(0, (currentTime / 300 % 2 == 0) ? color_wifi_connecting : color_off);
        statusPixel.show();
        delay(1000); 
        return; 
    }

    // 1. Check for Incoming Audio Packets from UDP (for Speaker) - listens on udpLocalPort
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
        IPAddress remoteIp = udp.remoteIP();
        int remotePort = udp.remotePort(); // Python server's source port (usually random)
        // Process only if it comes from the expected server and audio port (for security)
        // But we are keeping it simple for now.
        DEBUG_PRINT(2, "UDP_RX_AUDIO: Received %d byte UDP packet (for audio) from %s:%d.\n", packetSize, remoteIp.toString().c_str(), remotePort);

        int len = udp.read(incomingAudioBuffer, min(packetSize, INCOMING_AUDIO_BUFFER_SIZE));
        if (len > 0) {
            if (!isPlayingAudio) {
                DEBUG_PRINT(1, "I2S_SPK: Audio playback starting. Initializing I2S_NUM_1...\n");
                esp_err_t err_zero_bfr_play = i2s_zero_dma_buffer(I2S_NUM_1);
                if (err_zero_bfr_play != ESP_OK) DEBUG_PRINT(1, "I2S_SPK: WARNING: i2s_zero_dma_buffer error before playback: %s\n", esp_err_to_name(err_zero_bfr_play));

                esp_err_t err_start = i2s_start(I2S_NUM_1);
                if (err_start != ESP_OK) {
                    Serial.printf(PSTR("[%s] I2S_SPK: ERROR: Failed to start I2S_NUM_1! Error: %s\n"), millisToTimeStr(), esp_err_to_name(err_start));
                } else {
                    DEBUG_PRINT(2, "I2S_SPK: I2S_NUM_1 started successfully.\n");
                    isPlayingAudio = true;
                }
            }
            
            if(isPlayingAudio) {
                lastAudioPacketTime = currentTime;
                size_t bytes_written;
                // DEBUG_PRINT(2, "I2S_SPK: Writing %d bytes of received audio data to I2S_NUM_1...\n", len); // Very frequent log
                esp_err_t err_write = i2s_write(I2S_NUM_1, incomingAudioBuffer, len, &bytes_written, pdMS_TO_TICKS(i2sWriteTimeoutMs));

                if (err_write != ESP_OK) {
                    Serial.printf(PSTR("[%s] I2S_SPK: ERROR: Error writing to I2S_NUM_1! Error: %s\n"), millisToTimeStr(), esp_err_to_name(err_write));
                }
                if (bytes_written < len) {
                    DEBUG_PRINT(1, "I2S_SPK: WARNING: Not all audio data could be written! Requested: %d, Written: %d (Timeout: %lu ms)\n", len, bytes_written, i2sWriteTimeoutMs);
                }
            }
        } else {
            DEBUG_PRINT(1, "UDP_RX_AUDIO: WARNING: UDP packet received but readable data length is 0.\n");
        }
    }

    // 2. Check Audio Playback Status and Timeout
    if (isPlayingAudio && (currentTime - lastAudioPacketTime > audioTimeoutMs)) {
        DEBUG_PRINT(1, "I2S_SPK: Audio stream timed out (%lu ms). Stopping I2S_NUM_1.\n", audioTimeoutMs);
        isPlayingAudio = false;
        esp_err_t err_zero_after_play = i2s_zero_dma_buffer(I2S_NUM_1);
        if (err_zero_after_play != ESP_OK) DEBUG_PRINT(1, "I2S_SPK: WARNING: i2s_zero_dma_buffer error after timeout: %s\n", esp_err_to_name(err_zero_after_play));

        esp_err_t err_stop = i2s_stop(I2S_NUM_1);
        if (err_stop != ESP_OK) {
            Serial.printf(PSTR("[%s] I2S_SPK: ERROR: Failed to stop I2S_NUM_1! Error: %s\n"), millisToTimeStr(), esp_err_to_name(err_stop));
        } else {
            DEBUG_PRINT(2, "I2S_SPK: I2S_NUM_1 stopped successfully (timeout).\n");
        }
    }

    // 3. Read and Process MQ2 Gas Sensor (at specific intervals)
    if (currentTime - lastMq2Check >= mq2CheckInterval) {
        lastMq2Check = currentTime;
        int gasValue = analogRead(MQ2_PIN); // Read instantaneous gas value
        // DEBUG_PRINT(2, "MQ2: Gas Sensor Value: %d (Threshold: %d)\n", gasValue, threshold); // Can be very frequent log, exists in gasSend
        
        if (Blynk.connected()) {
            Blynk.virtualWrite(V0, gasValue);
        }

        bool previous_gas_alert_active = gas_alert_active; 
        if (gasValue > threshold) {
            gas_alert_active = true;
            if (!previous_gas_alert_active) { 
                DEBUG_PRINT(1, "MQ2: DANGER! Gas Detected! Value: %d\n", gasValue);
                if (Blynk.connected()) Blynk.logEvent("gas_warning", "Danger! Gas Detected! Sensor Value: " + String(gasValue));
                sendTelegramMessage("DANGER! Gas Detected on ESP32! Value: " + String(gasValue));
            }
            digitalWrite(BUZZER_PIN, LOW); 
        } else { 
            gas_alert_active = false;
            if (previous_gas_alert_active) { 
                DEBUG_PRINT(1, "MQ2: Gas level returned to normal. Value: %d\n", gasValue);
            }
            digitalWrite(BUZZER_PIN, HIGH);
        }
    }

    // 4. NEW: Send Gas Data to Python Server (at specific intervals)
    if (WiFi.status() == WL_CONNECTED && (currentTime - lastGasSendTime >= gasSendInterval)) {
        lastGasSendTime = currentTime;
        int currentGasValue = analogRead(MQ2_PIN); // Read gas data again just before sending

        String gasDataPayload = "gas:" + String(currentGasValue) + ",alert:" + String(gas_alert_active ? "true" : "false");
        
        // Send to Python server's GAS listening port using udp object
        udp.beginPacket(udpAddress, udpGasTargetPort); 
        udp.print(gasDataPayload);
        if (udp.endPacket()) {
            DEBUG_PRINT(2, "UDP_TX_GAS: Gas data sent: \"%s\" -> %s:%d\n", gasDataPayload.c_str(), udpAddress, udpGasTargetPort);
        } else {
            DEBUG_PRINT(1, "UDP_TX_GAS: ERROR: Gas data UDP packet could not be sent!\n");
        }
    }

    // 5. Update LED Status and (if not playing audio) Send Microphone Data
    uint32_t current_led_color = color_off;

    if (gas_alert_active) {
        current_led_color = ((currentTime / 200) % 2 == 0) ? color_gas_alert : color_off;
    } else if (isPlayingAudio) {
        current_led_color = ((currentTime / 150) % 2 == 0) ? color_audio_playing : color_off;
    } else { 
        if(WiFi.status() == WL_CONNECTED) {
            current_led_color = color_mic_active; 
        } else { 
            current_led_color = ((currentTime / 500) % 2 == 0) ? color_wifi_connecting : color_off;
        }

        // When not playing audio and no gas alarm, send microphone data to Python server's AUDIO port
        size_t bytesRead;
        esp_err_t err_read = i2s_read(I2S_NUM_0, i2s_mic_read_buffer, I2S_MIC_READ_LEN, &bytesRead, pdMS_TO_TICKS(10)); // Short timeout
        if (err_read == ESP_OK && bytesRead > 0) {
            // DEBUG_PRINT(2, "I2S_MIC: %d bytes of microphone data read. Sending via UDP (audio)...\n", bytesRead); // Very frequent log
            udp.beginPacket(udpAddress, udpTargetPort); // Python server's AUDIO listening port
            size_t sentBytes = udp.write((uint8_t*)i2s_mic_read_buffer, bytesRead);
            if (!udp.endPacket()) {
                DEBUG_PRINT(1, "UDP_TX_AUDIO: WARNING: UDP audio packet sending error (endPacket)!\n");
            } else if (sentBytes < bytesRead) {
                DEBUG_PRINT(1, "UDP_TX_AUDIO: WARNING: UDP packet sent but not all bytes could be written! Read: %d, Sent: %d\n", bytesRead, sentBytes);
            }
        } else if (err_read != ESP_OK && err_read != ESP_ERR_TIMEOUT) { 
            Serial.printf(PSTR("[%s] I2S_MIC: ERROR: Error reading from I2S_NUM_0! Error: %s (%d)\n"), millisToTimeStr(), esp_err_to_name(err_read), err_read);
        }
    }
    
    if (statusPixel.getPixelColor(0) != current_led_color) {
        statusPixel.setPixelColor(0, current_led_color);
    }
    statusPixel.show();

    // delay(1); // To give CPU a breather in very fast loops, usually unnecessary.
}