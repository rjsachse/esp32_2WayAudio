#include <Arduino.h>
#include <ESP_I2S.h>
#include <esp_http_server.h>

#define I2S_BCK_PIN     26
#define I2S_WS_PIN      25
#define I2S_DATA_IN_PIN 22
#define I2S_DATA_OUT_PIN 23

ESP_I2S i2s;
httpd_handle_t server = NULL;
bool streaming = false;
TaskHandle_t streamingTaskHandle = NULL;
TaskHandle_t micRemHandle = NULL;
int fdWs = -1;
uint8_t* audioBuffer = NULL;
static size_t audioBytesUsed = 0;
static int totalSamples = 0;
static const uint8_t sampleWidth = sizeof(int16_t);
const size_t sampleBytes = DMA_BUFF_LEN * sampleWidth;
int16_t* sampleBuffer = NULL;
static const char* micLabels[2] =
uint8_t* wsBuffer = NULL;
size_t wsBufferLen = 0;
bool stopAudio = false;

void applyMicRemGain() {
  float gainFactor = (float)pow(2, micGain - MIC_GAIN_CENTER);
  int16_t* wsPtr = (int16_t*) wsBuffer;
  for (int i = 0; i < wsBufferLen / sizeof(int16_t); i++) wsPtr[i] = constrain(wsPtr[i] * gainFactor, SHRT_MIN, SHRT_MAX);
}

static void ampOutputRem() {
    static int bytesCtr = 0;
    // Apply gain to the remote mic data
    applyMicRemGain(); // Implement this function as needed
    size_t bytesWritten = 0;
    bytesWritten = i2s.write(wsBuffer, wsBufferLen);
    bytesCtr += bytesWritten;
    if (bytesCtr > 16 * 16000 / 5) { // Assuming 16-bit samples and 16kHz sample rate
        int16_t* wsPtr = (int16_t*) wsBuffer;
        bytesCtr = 0;
    }
    wsBufferLen = 0;
}

static void micRemTask(void* parameter) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (streaming) {
            ampOutputRem();
            sendAudio();
        }
    }
}

void remoteMicHandler(uint8_t* wsMsg, size_t wsMsgLen) {
    if (!stopAudio && !wsBufferLen) {
        memcpy(wsBuffer, wsMsg, wsMsgLen);
        wsBufferLen = wsMsgLen;
        if (micRemHandle != NULL) {
            xTaskNotifyGive(micRemHandle);
        } else {
            Serial.println("Reboot ESP to use remote mic");
        }
    }
}

void micTaskStatus() {
    wsBufferLen = 0;
    if (wsBuffer == NULL) {
        wsBuffer = (uint8_t*)malloc(128); // Adjust size as needed
        if (wsBuffer == NULL) {
            Serial.println("Failed to allocate memory for wsBuffer");
            return;
        }
    }
    xTaskCreate(micRemTask, "micRemTask", 4096, NULL, 1, &micRemHandle);
}

static esp_err_t wsHandler(httpd_req_t *req) {
    esp_err_t ret = ESP_OK;
    if (req->method == HTTP_GET) {
        if (fdWs != -1) {
            if (fdWs != httpd_req_to_sockfd(req)) {
                Serial.printf("Closing connection, as newer WebSocket on %u\n", httpd_req_to_sockfd(req));
                httpd_sess_trigger_close(server, fdWs);
            }
        }
        fdWs = httpd_req_to_sockfd(req);
        if (fdWs < 0) {
            Serial.println("Failed to get socket number");
            ret = ESP_FAIL;
        } else {
            Serial.printf("WebSocket connection: %d\n", fdWs);
        }
    } else {
        httpd_ws_frame_t wsPkt;
        uint8_t wsMsg[128];
        memset(&wsPkt, 0, sizeof(httpd_ws_frame_t));
        wsPkt.payload = wsMsg;
        ret = httpd_ws_recv_frame(req, &wsPkt, sizeof(wsMsg));
        if (ret == ESP_OK) {
            wsMsg[wsPkt.len] = 0; // Null-terminate the message
            if (wsPkt.type == HTTPD_WS_TYPE_BINARY && wsPkt.len) {
                appSpecificWsBinHandler(wsMsg, wsPkt.len);
            } else if (wsPkt.type == HTTPD_WS_TYPE_TEXT) {
                appSpecificWsHandler((const char*)wsMsg);
            } else if (wsPkt.type == HTTPD_WS_TYPE_CLOSE) {
                appSpecificWsHandler("X");
            }
        } else {
            Serial.printf("WebSocket receive failed with %s\n", esp_err_to_name(ret));
        }
    }
    return ret;
}

void appSpecificWsHandler(const char* wsMsg) {
    int wsLen = strlen(wsMsg) - 1;
    int controlVal = atoi(wsMsg + 1); // Skip first char
    switch ((char)wsMsg[0]) {
        case 'X':
            // Stop remote mic stream
            stopAudio = true;
            // Fall through to kill connection
        case 'K': 
            // Kill WebSocket connection
            if (fdWs != -1) {
                httpd_sess_trigger_close(server, fdWs);
                fdWs = -1;
            }
            break;
        default:
            Serial.printf("Unknown command %c\n", (char)wsMsg[0]);
            break;
    }
}

void appSpecificWsBinHandler(uint8_t* wsMsg, size_t wsMsgLen) {
    remoteMicHandler(wsMsg, wsMsgLen);
}

void start_websocket_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = wsHandler,
        .user_ctx = NULL,
        .is_websocket = true
    };

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &ws_uri);
    }
}


void wsAsyncAudioSend(int16_t* wsData, size_t bytesRead) {
  // websockets send function, used for async logging and status updates
  if (fdWs >= 0) {
    // send if connection active
    httpd_ws_frame_t wsPkt;                                        
    wsPkt.payload = (uint8_t*)wsData;
    wsPkt.len = bytesRead;
    wsPkt.type = HTTPD_WS_TYPE_BINARY;
    wsPkt.final = true;
    esp_err_t ret = httpd_ws_send_frame_async(httpServer, fdWs, &wsPkt);
    if (ret != ESP_OK) LOG_WRN("websocket send failed with %s", esp_err_to_name(ret));
  } // else ignore
}

static size_t micInput() {
  uint8_t gainFactor = pow(2, micGain - MIC_GAIN_CENTER);
  size_t bytesRead = I2Smic ? i2s.readBytes((char*)sampleBuffer, sampleBytes) : I2Spdm.readBytes((char*)sampleBuffer, sampleBytes);
  int samplesRead = bytesRead / sampleWidth;
  // apply preamp gain
  for (int i = 0; i < samplesRead; i++) {
    sampleBuffer[i] = constrain(sampleBuffer[i] * gainFactor, SHRT_MIN, SHRT_MAX);
  }
  if (doStreamCapture && !audioBytesUsed) memcpy(audioBuffer, sampleBuffer, bytesRead);
  return bytesRead;
}

void sendAudio() {
  // play buffer from mic direct to amp
  size_t bytesRead = micInput();
  if (bytesRead) wsAsyncAudioSend(sampleBuffer, bytesRead);
}

void setup() {
    Serial.begin(115200);

    // I2S configuration
    i2s.setPins(I2S_BCK_PIN, I2S_WS_PIN, I2S_DATA_OUT_PIN, I2S_DATA_IN_PIN);
    i2s.begin(I2S_PHILIPS_MODE, 16000, 16);

    // Start WebSocket server
    start_websocket_server();
}

void loop() {
    // Main loop can handle other tasks
}
