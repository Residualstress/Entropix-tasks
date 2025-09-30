#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>   // ESP32 Arduino 内置轻量HTTP服务器
#include <s3servo.h>

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid     = "Galaxy";
const char *password = "q1234568";

// ===========================
// Servo Settings
// ===========================                            
s3servo servo1;
const int SERVO_PIN = 44;       // XIAO ESP32S3 可用GPIO，与你给的一致
const int SERVO_MIN = 0;        // 允许的最小角度
const int SERVO_MAX = 180;      // 允许的最大角度

// ===========================
// HTTP Servers
// ===========================
// 摄像头示例里自带的 Web 服务器将跑在 80 端口（在 startCameraServer() 内）
// 我们再起一个 8080 端口的服务器用于舵机控制，避免端口冲突
WebServer servoServer(8080);

// 来自 camera 示例的函数声明
void startCameraServer();
void setupLedFlash(int pin);

// 小工具：添加通用CORS/JSON响应头
static void addCommonHeaders() {
  servoServer.sendHeader("Access-Control-Allow-Origin", "*");
  servoServer.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  servoServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  servoServer.sendHeader("Cache-Control", "no-store");
}

// 解析并限制角度
static int clampAngle(int a) {
  if (a < SERVO_MIN) a = SERVO_MIN;
  if (a > SERVO_MAX) a = SERVO_MAX;
  return a;
}

// /servo?angle=90
void handleServoSet() {
  addCommonHeaders();

  if (!servoServer.hasArg("angle")) {
    servoServer.send(400, "application/json", "{\"ok\":false,\"error\":\"missing angle query param\"}");
    return;
  }

  int angle = servoServer.arg("angle").toInt();
  angle = clampAngle(angle);

  servo1.write(angle);
  // 给个很短的 settle 时间（可选）
  delay(5);

  String res = String("{\"ok\":true,\"angle\":") + angle + "}";
  servoServer.send(200, "application/json", res);
}

// /servo/sweep?min=30&max=150&step=2&delay=22
void handleServoSweep() {
  addCommonHeaders();

  int aMin = servoServer.hasArg("min")   ? servoServer.arg("min").toInt()   : 30;
  int aMax = servoServer.hasArg("max")   ? servoServer.arg("max").toInt()   : 150;
  int step = servoServer.hasArg("step")  ? servoServer.arg("step").toInt()  : 2;
  int dly  = servoServer.hasArg("delay") ? servoServer.arg("delay").toInt() : 22;

  aMin = clampAngle(aMin);
  aMax = clampAngle(aMax);
  if (aMin > aMax) std::swap(aMin, aMax);
  if (step <= 0) step = 1;
  if (dly  <  1) dly  = 1;

  // 做一次往返
  for (int a = aMin; a <= aMax; a += step) {
    servo1.write(a);
    delay(dly);
  }
  for (int a = aMax; a >= aMin; a -= step) {
    servo1.write(a);
    delay(dly);
  }

  String res = String("{\"ok\":true,\"min\":") + aMin +
               ",\"max\":" + aMax +
               ",\"step\":" + step +
               ",\"delay\":" + dly + "}";
  servoServer.send(200, "application/json", res);
}

// CORS 预检
void handleOptions() {
  addCommonHeaders();
  servoServer.send(204);  // No Content
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // ====== Camera Config (注意：为了避免与舵机库冲突，修改了 LEDC 通道/定时器) ======
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_2;  // 原示例是 0，这里改为 2
  config.ledc_timer   = LEDC_TIMER_1;    // 原示例是 0，这里改为 1
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
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    // 继续运行也没有意义，直接返回
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }
#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  // ====== WiFi ======
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");

  // ====== Servo attach（在 WiFi 之后、HTTP 之前）======
  servo1.attach(SERVO_PIN);  // s3servo 默认 50Hz，满足大多数舵机

  // ====== 启动摄像头 Web 服务器（端口 80）======
  startCameraServer();

  // ====== 舵机控制 HTTP 服务器（端口 8080）======
  // 路由：设置角度
  servoServer.on("/servo", HTTP_GET, handleServoSet);
  // 路由：扫角测试
  servoServer.on("/servo/sweep", HTTP_GET, handleServoSweep);
  // 预检
  servoServer.onNotFound([]() {
    addCommonHeaders();
  servoServer.send(404, "application/json", "{\"ok\":false,\"error\":\"not found\"}");
  });
  servoServer.on("/", HTTP_OPTIONS, handleOptions);
  servoServer.on("/servo", HTTP_OPTIONS, handleOptions);
  servoServer.on("/servo/sweep", HTTP_OPTIONS, handleOptions);

  servoServer.begin();

  // ====== 打印访问提示 ======
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  Serial.print("Servo API Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println(":8080/servo?angle=90' to set angle");
}

void loop() {
  // 处理 8080 端口的 HTTP 请求
  servoServer.handleClient();
  delay(1);
}
