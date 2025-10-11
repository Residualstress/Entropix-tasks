import struct
import threading
import serial
import math



class UWB360Receiver:
    def __init__(self, port, packet_cb=None, custom_filter=None):
        """
        :param port: 串口号
        :param packet_cb: 回调函数 signature: cb(parsed_dict)，返回结果带 xyz
        """
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.filter = custom_filter
        self.state = "WAIT_HEADER"
        self.buffer = bytearray()
        self.header_count = 0
        self.packet_length = 0
        self.on_packet = packet_cb
        self._running = False
        self._thread = None

    def start(self):
        """启动独立线程接收数据"""
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self.read_loop, daemon=True)
            self._thread.start()

    def stop(self):
        """停止线程"""
        self._running = False
        if self._thread:
            self._thread.join()
            self._thread = None

    def read_loop(self):
        """串口读取循环，状态机解包"""
        while self._running:
            byte = self.ser.read(1)
            if not byte:
                continue
            b = byte[0]

            if self.state == "WAIT_HEADER":
                if b == 0xFF:
                    self.header_count += 1
                    self.buffer.append(b)
                    if self.header_count == 4:
                        self.state = "READ_LEN"
                else:
                    self.header_count = 0
                    self.buffer.clear()

            elif self.state == "READ_LEN":
                self.buffer.append(b)
                if len(self.buffer) == 6:  # 4B header + 2B length
                    self.packet_length = struct.unpack_from(">H", self.buffer, 4)[0]
                    self.state = "READ_PAYLOAD"

            elif self.state == "READ_PAYLOAD":
                self.buffer.append(b)
                if len(self.buffer) == self.packet_length:
                    self._handle_packet(self.buffer)
                    # 重置状态机
                    self.state = "WAIT_HEADER"
                    self.buffer.clear()
                    self.header_count = 0

    def _handle_packet(self, packet: bytes):
        """异或校验 + 解析 payload + 回调"""
        calc_xor = 0
        for bb in packet[4:-1]:
            calc_xor ^= bb
        if calc_xor != packet[-1]:
            print("校验失败，丢弃")
            return

        payload = packet[6:-1]  # 跳过 header+len，去掉 xor
        parsed = self.parse_payload(payload)
        if parsed:
            # 转换为 xyz 坐标
            x,y,z = self.to_xyz(parsed)
            if self.filter:
                x,y = self.filter.update(x,y)
            if self.on_packet:
                try:
                    self.on_packet((x,y,z))
                except Exception as e:
                    print("回调处理异常:", e)

    def parse_payload(self, payload: bytes) -> dict:
        """解析 payload 字段"""
        fmt = ">HHHIIIhhHH4s"
        size = struct.calcsize(fmt)
        if len(payload) < size:
            print("payload 长度不足:", len(payload))
            return None
        (
            seq_id, req_cmd, version,
            anchor_id, tag_id, dist,
            azimuth, elevation,
            tag_status, batch_sn,
            reserve
        ) = struct.unpack(fmt, payload[:size])

        return {
            "SequenceID": seq_id,
            "RequestCommand": hex(req_cmd),
            "VersionID": hex(version),
            "AnchorID": anchor_id,
            "TagID": tag_id,
            "Distance_cm": dist,
            "Azimuth_deg": azimuth,
            "Elevation_deg": elevation,
            "TagStatus": tag_status,
            "BatchSn": batch_sn,
            "Reserve_raw": reserve
        }

    @staticmethod
    def to_xyz(parsed: dict):
        """根据距离、方位角、仰角计算 xyz 坐标"""
        r = parsed["Distance_cm"] / 100.0  # 转 m
        az = math.radians(parsed["Azimuth_deg"])
        el = math.radians(parsed["Elevation_deg"])
        x = r * math.cos(el) * math.cos(az)
        y = r * math.cos(el) * math.sin(az)
        z = r * math.sin(el)
        return x, y, z

if __name__ == "__main__":

    def packet_cb(pkt):
        print(pkt)

    parser = UWB360Receiver("/dev/ttyUSB0", packet_cb=packet_cb)
    parser.start()

    # 主线程可以继续做别的事情
    import time
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        parser.stop()
