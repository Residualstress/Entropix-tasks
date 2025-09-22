#include "esp32_camera.h"
#include "mcp_server.h"
#include "display.h"
#include "board.h"
#include "system_info.h"
#include "websocket_protocol.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <img_converters.h>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <thread>

#define TAG "Esp32Camera"

// 线程同步队列
QueueHandle_t jpeg_queue = nullptr;

Esp32Camera::Esp32Camera(const camera_config_t& config) {
    config_copy_ = config;
    esp_err_t err = esp_camera_init(&config_copy_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }
    inited_ = true;

    sensor_t *s = esp_camera_sensor_get(); 
    if (s) {
        ESP_LOGI(TAG, "Camera sensor initialized successfully");
        ESP_LOGI(TAG, "Sensor ID: PID=0x%04X, VER=0x%04X", s->id.PID, s->id.VER);

        if (s->id.PID == GC0308_PID) {
            ESP_LOGI(TAG, "GC0308 sensor detected, setting mirror");
            s->set_hmirror(s, 0);  // 这里控制摄像头镜像 写1镜像 写0不镜像
        } else {
            ESP_LOGW(TAG, "Unknown sensor PID: 0x%04X", s->id.PID);
        }

        // 配置摄像头参数
        s->set_framesize(s, config_copy_.frame_size);
        s->set_quality(s, 14);  // 设置JPEG质量（1-12）
    } else {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return;
    }

    // 初始化预览图像内存
    memset(&preview_image_, 0, sizeof(preview_image_));
    preview_image_.header.magic = LV_IMAGE_HEADER_MAGIC;
    preview_image_.header.cf = LV_COLOR_FORMAT_RGB565;
    preview_image_.header.flags = 0;

    // 根据分辨率配置图像参数
    switch (config.frame_size) {
        case FRAMESIZE_SVGA:
            preview_image_.header.w = 800;
            preview_image_.header.h = 600;
            break;
        case FRAMESIZE_VGA:
            preview_image_.header.w = 640;
            preview_image_.header.h = 480;
            break;
        case FRAMESIZE_QQVGA:
            preview_image_.header.w = 160;
            preview_image_.header.h = 120;
            break;
        case FRAMESIZE_QVGA:
            preview_image_.header.w = 320;
            preview_image_.header.h = 240;
            break;
        default:
            ESP_LOGE(TAG, "Unsupported frame size: %d", config.frame_size);
            preview_image_.data_size = 0;
            preview_image_.data = nullptr;
            return;
    }

    preview_image_.header.stride = preview_image_.header.w * 2;
    preview_image_.data_size = preview_image_.header.w * preview_image_.header.h * 2;
    preview_image_.data = (uint8_t*)heap_caps_malloc(preview_image_.data_size, MALLOC_CAP_SPIRAM);
    if (preview_image_.data == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate memory for preview image");
        return;
    }

    // 丢弃上电后的若干帧，避免初始化不稳定导致超时或 NO-SOI
    for (int i = 0; i < 8; ++i) {
        camera_fb_t* warm_fb = esp_camera_fb_get();
        if (warm_fb) {
            esp_camera_fb_return(warm_fb);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // 初始化JPEG编码队列
    jpeg_queue = xQueueCreate(40, sizeof(JpegChunk));
    if (jpeg_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create JPEG queue");
    }
}

// 析构函数
Esp32Camera::~Esp32Camera() {
    StopCamera();
    if (jpeg_queue != nullptr) {
        vQueueDelete(jpeg_queue);
    }
}

// 摄像头捕获图像并推送
void Esp32Camera::CaptureAndStream() {
    // 启动线程进行图像捕获与编码推送
    std::thread capture_thread([this]() {
        while (true) {
            if (!Capture()) {
                ESP_LOGE(TAG, "Failed to capture image");
                continue;
            }

            // 编码图像为JPEG格式并推送到服务器
            EncoderAndPush();
            vTaskDelay(pdMS_TO_TICKS(50));  // 控制推流间隔
        }
    });

    capture_thread.detach();  // 将线程分离，允许独立运行
}

// 捕获图像
bool Esp32Camera::Capture() {
    if (encoder_thread_.joinable()) {
        encoder_thread_.join();
    }

    int frames_to_get = 2;
    // Try to get a stable frame
    for (int i = 0; i < frames_to_get; i++) {
        if (fb_ != nullptr) {
            esp_camera_fb_return(fb_);
        }
        fb_ = esp_camera_fb_get();
        if (fb_ == nullptr) {
            ESP_LOGE(TAG, "Camera capture failed");
            return false;
        }
    }

    // 如果预览图片 buffer 为空，则跳过预览
    if (preview_image_.data_size == 0) {
        ESP_LOGW(TAG, "Skip preview because of unsupported frame size");
        return true;
    }
    if (preview_image_.data == nullptr) {
        ESP_LOGE(TAG, "Preview image data is not initialized");
        return true;
    }
    // 若为 JPEG 模式则跳过 RGB565 预览拷贝
    if (fb_->format == PIXFORMAT_RGB565) {
        auto display = Board::GetInstance().GetDisplay();
        if (display != nullptr) {
            auto src = (uint16_t*)fb_->buf;
            auto dst = (uint16_t*)preview_image_.data;
            size_t pixel_count = fb_->len / 2;
            for (size_t i = 0; i < pixel_count; i++) {
                dst[i] = __builtin_bswap16(src[i]);
            }
            display->SetPreviewImage(&preview_image_);
        }
    }
    return true;
}

// 编码并推送JPEG图像
void Esp32Camera::EncoderAndPush() {
    // 编码JPEG并推送到服务器
    encoder_thread_ = std::thread([this]() {
        frame2jpg_cb(fb_, 50, [](void* arg, size_t index, const void* data, size_t len) -> unsigned int {
            auto jpeg_queue = (QueueHandle_t)arg;
            JpegChunk chunk = {
                .data = (uint8_t*)heap_caps_aligned_alloc(16, len, MALLOC_CAP_SPIRAM),
                .len = len
            };
            memcpy(chunk.data, data, len);
            xQueueSend(jpeg_queue, &chunk, portMAX_DELAY);
            return len;
        }, jpeg_queue);
    });

    encoder_thread_.join();
}

// 推流开始
bool Esp32Camera::StartStreaming(int fps, int quality) {
    if (streaming_) {
        ESP_LOGW(TAG, "推流已经在运行中");
        return true;
    }
    
    if (!inited_) {
        ESP_LOGE(TAG, "摄像头未初始化，无法开始推流");
        return false;
    }

    streaming_ = true;
    ESP_LOGI(TAG, "开始推流 - FPS: %d, 质量: %d", fps, quality);
    return true;
}

// 推流停止
void Esp32Camera::StopStreaming() {
    if (!streaming_) {
        ESP_LOGW(TAG, "推流未在运行");
        return;
    }

    streaming_ = false;
    ESP_LOGI(TAG, "推流停止");
}
