# RTSP视频传输使用说明

## ESP32CAM_RTSP固件

在VSCode中安装PlatformIO插件，并打开esp32cam-rtsp目录，修改platformio.ini里使用的开发板型号，编译上传。https://github.com/rzeldent/esp32cam-rtsp

烧录后，可在串口中找到ip地址，打开ip地址后可以调整参数，获取视频流连接。

## 地面端

地面端可使用VLC打开RTSP流，或使用rtsp_receive.py收取视频流。