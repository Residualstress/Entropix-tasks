/*
  AprilTag demo with WebSocket push
  XIAO ESP32S3 + PSRAM
  - Push detections to clients in real-time via WebSocket
*/
#define CAMERA_MODEL_XIAO_ESP32S3
#include <Arduino.h>
#include "esp_camera.h"
#include "camera_pins.h"

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "apriltag.h"
#include "tag25h9.h"
#include "common/image_u8.h"
#include "common/zarray.h"

// WiFi
const char* WIFI_SSID = "Tian’s iPhone";
const char* WIFI_PASS = "wantangbellcat";

// WebServer
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Apriltag
apriltag_family_t *tf = nullptr;
apriltag_detector_t *td = nullptr;

// Shared data
String latest_json = "[]";
SemaphoreHandle_t json_mutex;

// Forward declaration
void apriltag_task(void *pvParameters);

void notifyClients(const String &msg){
    ws.textAll(msg); // broadcast to all connected WS clients
}

void setup() {
    Serial.begin(115200);
    delay(100);

    psramInit();
    Serial.print("Free PSRAM: "); Serial.println(ESP.getFreePsram());

    // Camera init (keep as your original)
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_QVGA;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count = 1;

    if(esp_camera_init(&config) != ESP_OK){
        Serial.println("Camera init failed");
        while(1) delay(1000);
    }

    sensor_t * s = esp_camera_sensor_get();
    s->set_brightness(s,0);
    s->set_contrast(s,0);
    s->set_saturation(s,0);
    s->set_whitebal(s,1);

    // WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    while(WiFi.status() != WL_CONNECTED){ delay(250); Serial.print("."); }
    Serial.println();
    Serial.print("IP: "); Serial.println(WiFi.localIP());

    // WebSocket event (not strictly needed here)
    ws.onEvent([](AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type,
                  void * arg, uint8_t *data, size_t len){
        if(type == WS_EVT_CONNECT) Serial.printf("WS client %u connected\n", client->id());
        if(type == WS_EVT_DISCONNECT) Serial.printf("WS client %u disconnected\n", client->id());
    });
    server.addHandler(&ws);
    server.begin();
    Serial.println("WebSocket server started at /ws");

    // Mutex
    json_mutex = xSemaphoreCreateMutex();

    // Apriltag init
    tf = tag25h9_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_sigma = 0;
    td->quad_decimate = 3.0;
    td->refine_edges = 0;
    td->decode_sharpening = 0.15;
    td->nthreads = 1;

    // Start detection task
    xTaskCreatePinnedToCore(apriltag_task, "apriltag_task", 32*1024, NULL, 1, NULL, 1);
}

String detections_to_json(zarray_t *detections){
    String out = "[";
    int n = zarray_size(detections);
    for(int i=0;i<n;i++){
        apriltag_detection_t *det;
        zarray_get(detections,i,&det);
        out += "{\"id\":" + String(det->id) + ",\"cx\":" + String(det->c[0],2) + ",\"cy\":" + String(det->c[1],2) + "}";
        if(i<n-1) out += ",";
    }
    out += "]";
    return out;
}

void apriltag_task(void *pvParameters){
    while(true){
        camera_fb_t * fb = esp_camera_fb_get();
        if(!fb){ vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        image_u8_t im = { .width=fb->width, .height=fb->height, .stride=fb->width, .buf=fb->buf };
        zarray_t *detections = apriltag_detector_detect(td,&im);

        String j = detections_to_json(detections);
        if(xSemaphoreTake(json_mutex, (TickType_t)5) == pdTRUE){
            latest_json = j;
            xSemaphoreGive(json_mutex);
        }

        if(zarray_size(detections)>0) notifyClients(j);

        apriltag_detections_destroy(detections);
        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

void loop(){}
