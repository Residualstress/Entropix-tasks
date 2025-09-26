#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
更鲁棒的 ESP32 MJPEG 转发服务：
- /mjpeg1, /mjpeg2: MJPEG 流
- /snapshot: 最新帧
- /ws: WebSocket 二进制 JPEG
- /health: 运行状态
使用方法：python rebroadcast_server.py --source http://10.201.171.123:81/stream --host 0.0.0.0 --port 8088
"""

import asyncio
import argparse
import time
import logging
from typing import Optional, Tuple

import aiohttp
from aiohttp import web

BOUNDARY_OUT = b"--frame"
DEFAULT_PUSH_FPS = 12

log = logging.getLogger("rebroadcast")


def setup_logging(level: str):
    lvl = getattr(logging, level.upper(), logging.INFO)
    logging.basicConfig(
        level=lvl,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


class MJPEGSource:
    def __init__(self, source_url: str, connect_timeout: float = 5.0, read_timeout: float = 0):
        self.source_url = source_url
        self.connect_timeout = connect_timeout
        self.read_timeout = read_timeout
        self.session: Optional[aiohttp.ClientSession] = None
        self._task: Optional[asyncio.Task] = None
        self._stop = asyncio.Event()

        self.latest_frame: Optional[bytes] = None
        self.latest_ts: float = 0.0
        self.stats = {
            "frames": 0,
            "last_error": "",
            "connected": False,
            "last_connect": 0.0,
        }

    async def start(self):
        if self._task:
            return
        self.session = aiohttp.ClientSession()
        self._task = asyncio.create_task(self._run())

    async def stop(self):
        self._stop.set()
        if self._task:
            await self._task
        if self.session:
            await self.session.close()

    async def _run(self):
        backoff = 1.0
        while not self._stop.is_set():
            self.stats["last_connect"] = time.time()
            try:
                timeout = aiohttp.ClientTimeout(
                    total=None,
                    sock_connect=self.connect_timeout,
                    sock_read=None if self.read_timeout <= 0 else self.read_timeout,
                )
                log.info(f"Connecting to source: {self.source_url}")
                async with self.session.get(self.source_url, timeout=timeout) as resp:
                    if resp.status != 200:
                        self.stats["last_error"] = f"HTTP {resp.status}"
                        log.warning(f"Source HTTP status: {resp.status}")
                        await asyncio.sleep(backoff)
                        backoff = min(backoff * 2, 15)
                        continue

                    ctype = resp.headers.get("Content-Type", "")
                    boundary = self._parse_boundary(ctype) or b'--123456789000000000000987654321'
                    log.info(f"Source connected. Parsed boundary: {boundary!r}")
                    self.stats["connected"] = True
                    self.stats["last_error"] = ""
                    backoff = 1.0

                    buf = b""
                    reader = resp.content
                    while not reader.at_eof() and not self._stop.is_set():
                        chunk = await reader.read(8192)
                        if not chunk:
                            await asyncio.sleep(0.01)
                            continue
                        buf += chunk
                        while True:
                            part, buf = self._pop_part(buf, boundary)
                            if part is None:
                                break
                            frame = self._extract_jpeg(part)
                            if frame:
                                self.latest_frame = frame
                                self.latest_ts = time.time()
                                self.stats["frames"] += 1
                    log.info("Source stream ended.")
            except (aiohttp.ClientConnectorError, asyncio.TimeoutError, aiohttp.ClientPayloadError) as e:
                self.stats["last_error"] = str(e)
                log.warning(f"Source error: {e}")
            except Exception as e:
                self.stats["last_error"] = f"Unexpected: {e}"
                log.exception("Unexpected error")
            finally:
                self.stats["connected"] = False
                if not self._stop.is_set():
                    await asyncio.sleep(backoff)
                    backoff = min(backoff * 2, 15)

    @staticmethod
    def _parse_boundary(content_type: str) -> Optional[bytes]:
        # e.g. multipart/x-mixed-replace;boundary=12345
        ct = (content_type or "").lower()
        if "multipart" not in ct or "boundary=" not in ct:
            return None
        b = ct.split("boundary=", 1)[1].strip()
        if b.startswith('"') and b.endswith('"'):
            b = b[1:-1]
        if not b.startswith("--"):
            b = "--" + b
        return b.encode("utf-8", "ignore")

    @staticmethod
    def _pop_part(buffer: bytes, boundary: bytes) -> Tuple[Optional[bytes], bytes]:
        """
        从 buffer 中取出一个完整的 part（含头+空行+body）。
        兼容首个 part 可能直接以 boundary 开始（无前置 \r\n）。
        """
        # boundary 可能在起始（buffer 开头），也可能在 \r\n 后
        idx0 = buffer.find(boundary)
        idx1 = buffer.find(b"\r\n" + boundary)

        if idx0 == -1 and idx1 == -1:
            # 未出现 boundary
            return None, buffer

        # 选更靠前的那个
        start_idx = idx0 if (idx0 != -1 and (idx1 == -1 or idx0 < idx1)) else idx1
        if start_idx == idx1:
            start_idx += 2  # 跳过前置 \r\n

        # 定位 header 结束
        header_end = buffer.find(b"\r\n\r\n", start_idx + len(boundary))
        if header_end == -1:
            # 头还不完整
            return None, buffer

        headers_blob = buffer[start_idx + len(boundary): header_end]
        clen = MJPEGSource._content_length_from_headers(headers_blob)
        body_start = header_end + 4

        if clen is not None:
            body_end = body_start + clen
            if len(buffer) < body_end:
                return None, buffer
            part = buffer[start_idx:body_end]
            remain = buffer[body_end:]
            return part, remain
        else:
            # 没 Content-Length，就找下一个 boundary 做结束
            next_idx = buffer.find(boundary, body_start)
            if next_idx == -1:
                # 可能还有 \r\n + boundary
                next_idx = buffer.find(b"\r\n" + boundary, body_start)
                if next_idx == -1:
                    return None, buffer
                part = buffer[start_idx:next_idx]
                remain = buffer[next_idx + 2:]  # 去掉前置 \r\n
            else:
                part = buffer[start_idx:next_idx]
                remain = buffer[next_idx:]
            return part, remain

    @staticmethod
    def _content_length_from_headers(headers_blob: bytes) -> Optional[int]:
        try:
            text = headers_blob.decode("iso-8859-1", "ignore")
            for line in text.split("\r\n"):
                if line.lower().startswith("content-length:"):
                    v = line.split(":", 1)[1].strip()
                    return int(v)
        except Exception:
            return None
        return None

    @staticmethod
    def _extract_jpeg(part: bytes) -> Optional[bytes]:
        # part = boundary + headers + \r\n\r\n + body (+ 可能结尾 \r\n)
        try:
            idx = part.find(b"\r\n\r\n")
            if idx == -1:
                return None
            body = part[idx + 4:]
            # 仅裁剪两端空白，避免截断真实数据
            return body.strip()
        except Exception:
            return None


async def mjpeg_handler(request: web.Request, source: MJPEGSource):
    fps = int(request.query.get("fps", DEFAULT_PUSH_FPS))
    fps = max(1, min(30, fps))
    interval = 1.0 / fps

    headers = {
        "Content-Type": "multipart/x-mixed-replace; boundary=frame",
        "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
        "Pragma": "no-cache",
        "Access-Control-Allow-Origin": "*",
        "Connection": "close",
    }
    resp = web.StreamResponse(status=200, headers=headers)
    await resp.prepare(request)
    log.info(f"Client connected: {request.path} from {request.remote}, fps={fps}")

    last_sent_ts = 0.0
    try:
        while True:
            frame = source.latest_frame
            ts = source.latest_ts
            if not frame:
                await asyncio.sleep(0.05)
                continue
            if ts <= last_sent_ts:
                await asyncio.sleep(0.005)
                continue
            last_sent_ts = ts

            await resp.write(BOUNDARY_OUT + b"\r\n")
            await resp.write(b"Content-Type: image/jpeg\r\n")
            await resp.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
            await resp.write(frame)
            await resp.write(b"\r\n")

            await asyncio.sleep(interval)
    except (asyncio.CancelledError, ConnectionResetError, ConnectionAbortedError):
        pass
    except Exception:
        log.exception("Error while streaming to client")
    finally:
        log.info(f"Client disconnected: {request.path} from {request.remote}")
        try:
            await resp.write_eof()
        except Exception:
            pass
    return resp


async def snapshot_handler(request: web.Request, source: MJPEGSource):
    if not source.latest_frame:
        return web.Response(status=503, text="no frame yet")
    return web.Response(
        status=200,
        body=source.latest_frame,
        headers={
            "Content-Type": "image/jpeg",
            "Cache-Control": "no-store",
            "Access-Control-Allow-Origin": "*",
        },
    )


async def ws_handler(request: web.Request, source: MJPEGSource):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    fps = int(request.query.get("fps", DEFAULT_PUSH_FPS))
    fps = max(1, min(30, fps))
    interval = 1.0 / fps
    last_sent_ts = 0.0
    try:
        while True:
            # 定时取帧并推送
            await asyncio.sleep(interval)
            frame = source.latest_frame
            ts = source.latest_ts
            if frame and ts > last_sent_ts:
                await ws.send_bytes(frame)
                last_sent_ts = ts
    except Exception:
        pass
    finally:
        await ws.close()
    return ws


async def health_handler(request: web.Request, source: MJPEGSource):
    data = {
        "connected": source.stats["connected"],
        "frames": source.stats["frames"],
        "last_error": source.stats["last_error"],
        "last_connect": source.stats["last_connect"],
        "has_frame": source.latest_frame is not None,
        "source": source.source_url,
    }
    return web.json_response(data, headers={"Access-Control-Allow-Origin": "*"})

# ------------------- app -------------------

async def init_app(args):
    source = MJPEGSource(args.source)
    await source.start()

    app = web.Application()
    app["source"] = source

    app.router.add_get("/mjpeg1", lambda r: mjpeg_handler(r, source))
    app.router.add_get("/mjpeg2", lambda r: mjpeg_handler(r, source))
    app.router.add_get("/snapshot", lambda r: snapshot_handler(r, source))
    app.router.add_get("/ws",       lambda r: ws_handler(r, source))
    app.router.add_get("/health",   lambda r: health_handler(r, source))

    async def on_cleanup(app_):
        await source.stop()
    app.on_cleanup.append(on_cleanup)

    return app


def main():
    parser = argparse.ArgumentParser(description="ESP32 MJPEG Rebroadcast Server")
    parser.add_argument("--source", required=True, help="上游 MJPEG URL，例如 http://<ESP32_IP>/stream")
    parser.add_argument("--host", default="0.0.0.0", help="本机监听地址")
    parser.add_argument("--port", type=int, default=8088, help="本机监听端口")
    parser.add_argument("--log", default="info", help="日志级别：debug/info/warn/error")
    args = parser.parse_args()

    setup_logging(args.log)
    web.run_app(init_app(args), host=args.host, port=args.port)


if __name__ == "__main__":
    main()