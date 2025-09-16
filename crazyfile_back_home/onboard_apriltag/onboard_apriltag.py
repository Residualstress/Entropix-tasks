import threading
import json
import websocket  # pip install websocket-client

class ESP32AprilTagWS:
    def __init__(self, esp_ip, callback):
        self.ws_url = f"ws://{esp_ip}/ws"
        self.callback = callback
        self._ws = None
        self._thread = None
        self._stop_event = threading.Event()

    def start(self):
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._ws:
            self._ws.close()
        if self._thread:
            self._thread.join()

    def _on_message(self, ws, message):
        try:
            data = json.loads(message)
            self.callback(data)
        except Exception as e:
            print("JSON parse error:", e)

    def _run(self):
        while not self._stop_event.is_set():
            try:
                self._ws = websocket.WebSocketApp(
                    self.ws_url,
                    on_message=self._on_message,
                )
                self._ws.run_forever()
            except Exception as e:
                print("WebSocket reconnect in 1s:", e)
                import time; time.sleep(1)

# ---------------- 使用示例 ----------------
if __name__ == "__main__":
    import time
    ts_last = time.time()
    def my_callback(detections):
        global ts_last
        print(f'Received in {time.time() - ts_last}')
        ts_last = time.time()
        if detections:
            print("Detections:", detections)
        else:
            print("No tags detected.")

    client = ESP32AprilTagWS("172.20.10.9", my_callback)
    client.start()

    try:
        while True:
            import time; time.sleep(1)
    except KeyboardInterrupt:
        client.stop()
