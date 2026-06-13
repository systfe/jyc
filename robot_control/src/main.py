#!/usr/bin/env python3

"""
RTSP 延迟测试程序。

用法：
    python3 main.py

测试方法：
    1. 程序会打开 `latency_clock` 大时钟窗口。
    2. 把摄像头对准这个窗口。
    3. 在 `rtsp_latency_test` 画面中比较：
       - 摄像头画面里拍到的时钟
       - 画面左上角叠加的 NOW 时间
    两者差值就是肉眼可见的真实端到端延迟。
"""

import os
import socket
import sys
import threading
import time
from datetime import datetime
from typing import Iterable, List, Optional, Tuple
from urllib.parse import quote, urlparse

import cv2
import numpy as np

try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
except Exception:
    Gst = None


# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------

CAMERA_IP = "192.168.10.100"
NETWORK_PREFIX = "192.168.10."
SCAN_START = 100
SCAN_END = 254

RTSP_PORT = 554
USERNAME = "admin"
PASSWORD = "zxcvbnm12"

# 留空时会按 RTSP_PATHS 顺序尝试，优先测试子码流。
# 如果要强制只测试某个地址，再填完整 URL。
RTSP_URL = ""

RTSP_PATHS = [
    "/stream1",
    "/stream2",
    "/Streaming/Channels/102",
    "/cam/realmonitor?channel=1&subtype=1",
    "/live",
    "/h264",
    "/h265",
    "/ch1/main/av_stream",
    "/Streaming/Channels/101",
    "/cam/realmonitor?channel=1&subtype=0",
]

# 有些相机的 H264 RTSP 会固定带 0.5~1s 编码缓存，但 MJPEG/HTTP 流
# 可能更低。如果打不开会自动跳过。
HTTP_PATHS = [
    "/video",
    "/mjpeg",
    "/mjpg/video.mjpg",
    "/videostream.cgi",
    "/cgi-bin/mjpg/video.cgi?channel=1&subtype=1",
    "/ISAPI/Streaming/channels/102/httpPreview",
]

# GStreamer 在这台相机上可能卡在 RTSP 握手。默认先用已验证能打开的
# OpenCV/FFmpeg 低延迟路径；需要单独试 GStreamer 时把它改成 True。
TRY_GSTREAMER = False

BACKENDS_TO_TRY = []
if TRY_GSTREAMER:
    BACKENDS_TO_TRY.extend([
        ("gst_udp_latency0", "gstreamer", {"protocols": "udp", "latency_ms": 0}),
        ("gst_tcp_latency0", "gstreamer", {"protocols": "tcp", "latency_ms": 0}),
    ])

BACKENDS_TO_TRY.extend([
    ("opencv_http_plain", "opencv_http", {}),
    ("opencv_udp_low_delay", "opencv", {
        "rtsp_transport": "udp",
        "fflags": "nobuffer",
        "flags": "low_delay",
        "max_delay": "0",
        "reorder_queue_size": "0",
        "stimeout": "2000000",
    }),
    ("opencv_tcp_low_delay", "opencv", {
        "rtsp_transport": "tcp",
        "fflags": "nobuffer",
        "flags": "low_delay",
        "max_delay": "0",
        "reorder_queue_size": "0",
        "stimeout": "2000000",
    }),
    ("opencv_tcp_compatible", "opencv", {
        "rtsp_transport": "tcp",
        "stimeout": "5000000",
        "max_delay": "500000",
    }),
    ("opencv_plain", "opencv", {}),
])

SCAN_PORT_TIMEOUT_S = 0.08
OPEN_TIMEOUT_S = 3.0
RECONNECT_DELAY_S = 0.5

SHOW_PREVIEW = True
SHOW_CLOCK_WINDOW = True
PREVIEW_WINDOW = "rtsp_latency_test"
CLOCK_WINDOW = "latency_clock"

OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 640
FPS_LIMIT = 0.0  # 0 表示来一帧显示一帧。

PUBLISH_ROS_IMAGE = False
ROS_IMAGE_TOPIC = "/robot/image_raw"
FRAME_ID = "fisheye_camera"


# ---------------------------------------------------------------------------
# 地址工具
# ---------------------------------------------------------------------------

def build_rtsp_url(ip: str, path: str) -> str:
    if path and not path.startswith("/"):
        path = "/" + path

    auth = ""
    if USERNAME:
        auth = quote(USERNAME, safe="")
        if PASSWORD:
            auth += ":" + quote(PASSWORD, safe="")
        auth += "@"

    return f"rtsp://{auth}{ip}:{RTSP_PORT}{path}"


def build_http_url(ip: str, path: str) -> str:
    if path and not path.startswith("/"):
        path = "/" + path

    auth = ""
    if USERNAME:
        auth = quote(USERNAME, safe="")
        if PASSWORD:
            auth += ":" + quote(PASSWORD, safe="")
        auth += "@"

    return f"http://{auth}{ip}{path}"


def is_rtsp_port_open(ip: str) -> bool:
    try:
        with socket.create_connection((ip, RTSP_PORT), timeout=SCAN_PORT_TIMEOUT_S):
            return True
    except OSError:
        return False


def candidate_ips() -> List[str]:
    if CAMERA_IP:
        return [CAMERA_IP]

    print(
        f"[RTSP] CAMERA_IP 为空，正在扫描 "
        f"{NETWORK_PREFIX}{SCAN_START}..{SCAN_END}:{RTSP_PORT}"
    )
    found = []
    for host in range(SCAN_START, SCAN_END + 1):
        ip = f"{NETWORK_PREFIX}{host}"
        if is_rtsp_port_open(ip):
            print(f"[RTSP] 发现开放的 RTSP 端口: {ip}:{RTSP_PORT}")
            found.append(ip)
    return found


def candidate_urls() -> List[str]:
    if RTSP_URL:
        return [RTSP_URL]

    urls = []
    for ip in candidate_ips():
        for path in HTTP_PATHS:
            urls.append(build_http_url(ip, path))
        for path in RTSP_PATHS:
            urls.append(build_rtsp_url(ip, path))
    return urls


# ---------------------------------------------------------------------------
# 图像工具
# ---------------------------------------------------------------------------

def now_text() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def resize_frame(frame):
    if OUTPUT_WIDTH <= 0 and OUTPUT_HEIGHT <= 0:
        return frame

    if OUTPUT_WIDTH > 0 and OUTPUT_HEIGHT > 0:
        size = (OUTPUT_WIDTH, OUTPUT_HEIGHT)
    elif OUTPUT_WIDTH > 0:
        scale = OUTPUT_WIDTH / frame.shape[1]
        size = (OUTPUT_WIDTH, max(1, int(frame.shape[0] * scale)))
    else:
        scale = OUTPUT_HEIGHT / frame.shape[0]
        size = (max(1, int(frame.shape[1] * scale)), OUTPUT_HEIGHT)

    return cv2.resize(frame, size, interpolation=cv2.INTER_AREA)


def short_url(url: str) -> str:
    parsed = urlparse(url)
    path = parsed.path or "/"
    if parsed.query:
        path += "?" + parsed.query
    return f"{parsed.hostname}:{parsed.port or RTSP_PORT}{path}"


def draw_text_panel(frame, lines: List[str]) -> None:
    x = 12
    y = 28
    line_h = 28
    panel_h = 12 + line_h * len(lines)
    panel_w = min(frame.shape[1], 760)
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (panel_w, panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0.0, frame)

    for line in lines:
        cv2.putText(
            frame,
            line,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        y += line_h


def make_clock_image() -> np.ndarray:
    image = np.zeros((360, 900, 3), dtype=np.uint8)
    text = now_text()
    cv2.putText(
        image,
        text,
        (45, 205),
        cv2.FONT_HERSHEY_SIMPLEX,
        3.4,
        (0, 255, 255),
        8,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        "将摄像头对准本窗口，对比画面中的时间和 NOW 叠加时间。",
        (55, 300),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (180, 180, 180),
        2,
        cv2.LINE_AA,
    )
    return image


# ---------------------------------------------------------------------------
# ROS 发布，可选
# ---------------------------------------------------------------------------

def make_image_msg(frame, node):
    from sensor_msgs.msg import Image

    msg = Image()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = FRAME_ID
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = frame.shape[1] * 3
    msg.data = frame.tobytes()
    return msg


def start_ros_node():
    if not PUBLISH_ROS_IMAGE:
        return None, None

    import rclpy
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node("rtsp_latency_test_publisher")
    publisher = node.create_publisher(Image, ROS_IMAGE_TOPIC, 1)
    node.get_logger().info(f"正在发布 RTSP 图像到 {ROS_IMAGE_TOPIC}")
    return node, publisher


def stop_ros_node(node) -> None:
    if node is None:
        return

    import rclpy

    node.destroy_node()
    rclpy.shutdown()


# ---------------------------------------------------------------------------
# 采集后端
# ---------------------------------------------------------------------------

class CaptureBackend:
    def read(self) -> Optional[np.ndarray]:
        raise NotImplementedError

    def close(self) -> None:
        pass


class OpenCvCapture(CaptureBackend):
    def __init__(self, url: str, profile_name: str, options: dict):
        self.url = url
        self.profile_name = profile_name
        if options:
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "|".join(
                f"{key};{value}" for key, value in options.items()
            )
        else:
            os.environ.pop("OPENCV_FFMPEG_CAPTURE_OPTIONS", None)
        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def is_opened(self) -> bool:
        return self.cap.isOpened()

    def read(self) -> Optional[np.ndarray]:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        return resize_frame(frame)

    def close(self) -> None:
        self.cap.release()


class GStreamerCapture(CaptureBackend):
    def __init__(self, url: str, profile_name: str, options: dict):
        if Gst is None:
            raise RuntimeError("GStreamer Python 绑定不可用")
        Gst.init(None)
        self.url = url
        self.profile_name = profile_name
        self.pipeline = None
        self.sink = None
        self._open(options)

    def _open(self, options: dict) -> None:
        protocols = options.get("protocols", "udp")
        latency_ms = int(options.get("latency_ms", 0))
        pipeline = (
            f"rtspsrc location=\"{self.url}\" protocols={protocols} "
            f"latency={latency_ms} drop-on-latency=true do-retransmission=false ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=false max-buffers=1 drop=true sync=false"
        )
        self.pipeline = Gst.parse_launch(pipeline)
        self.sink = self.pipeline.get_by_name("sink")
        self.pipeline.set_state(Gst.State.PLAYING)

    def wait_opened(self) -> bool:
        deadline = time.monotonic() + OPEN_TIMEOUT_S
        while time.monotonic() < deadline:
            frame = self.read(timeout_ms=200)
            if frame is not None:
                self._first_frame = frame
                return True
        return False

    def read(self, timeout_ms: int = 50) -> Optional[np.ndarray]:
        if hasattr(self, "_first_frame"):
            frame = self._first_frame
            del self._first_frame
            return frame

        sample = self.sink.emit("try-pull-sample", timeout_ms * Gst.MSECOND)
        if sample is None:
            return None

        caps = sample.get_caps()
        if caps is None or caps.get_size() == 0:
            return None

        structure = caps.get_structure(0)
        width = structure.get_value("width")
        height = structure.get_value("height")
        buf = sample.get_buffer()
        ok, map_info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None

        try:
            frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape((height, width, 3)).copy()
            return resize_frame(frame)
        finally:
            buf.unmap(map_info)

    def close(self) -> None:
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        self.pipeline = None
        self.sink = None


class LatestFrameCapture:
    def __init__(self, urls: List[str]):
        self.urls = urls
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._backend: Optional[CaptureBackend] = None
        self._backend_name = ""
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_time = 0.0
        self._latest_seq = 0

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        if self._backend is not None:
            self._backend.close()

    def latest(self) -> Tuple[Optional[np.ndarray], float, int, str]:
        with self._lock:
            if self._latest_frame is None:
                return None, 0.0, self._latest_seq, self._backend_name
            return self._latest_frame.copy(), self._latest_time, self._latest_seq, self._backend_name

    def _run(self) -> None:
        while not self._stop_event.is_set():
            self._backend, self._backend_name = self._open_backend()
            if self._backend is None:
                time.sleep(RECONNECT_DELAY_S)
                continue

            while not self._stop_event.is_set():
                frame = self._backend.read()
                if frame is None:
                    continue

                with self._lock:
                    self._latest_frame = frame
                    self._latest_time = time.monotonic()
                    self._latest_seq += 1

            self._backend.close()
            self._backend = None

    def _open_backend(self) -> Tuple[Optional[CaptureBackend], str]:
        for url in self.urls:
            for profile_name, backend_type, options in BACKENDS_TO_TRY:
                print(f"[RTSP] 正在尝试 ({profile_name}): {url}", flush=True)
                try:
                    if backend_type == "gstreamer":
                        backend = GStreamerCapture(url, profile_name, options)
                        opened = backend.wait_opened()
                    else:
                        backend = OpenCvCapture(url, profile_name, options)
                        opened = backend.is_opened()
                        if opened:
                            first = backend.read()
                            opened = first is not None
                            if opened:
                                with self._lock:
                                    self._latest_frame = first
                                    self._latest_time = time.monotonic()
                                    self._latest_seq += 1

                    if opened:
                        backend_label = f"{profile_name} {short_url(url)}"
                        print(f"[RTSP] 已打开: {url}, profile={profile_name}", flush=True)
                        return backend, backend_label
                    backend.close()
                except Exception as exc:
                    print(f"[RTSP] {profile_name} 失败: {exc}", flush=True)

        return None, ""


# ---------------------------------------------------------------------------
# 主循环
# ---------------------------------------------------------------------------

def main() -> int:
    cv2.setNumThreads(1)
    urls = candidate_urls()
    if not urls:
        print("[RTSP] 没有找到可用的相机候选地址。")
        return 1

    print("[TEST] 请把摄像头对准 `latency_clock` 窗口。")
    print("[TEST] 对比摄像头画面里的时钟和左上角 NOW 叠加时间。")
    print("[TEST] 按 q/Esc 退出，按 s 保存当前预览到 /tmp/rtsp_latency_test.jpg。")

    node, publisher = start_ros_node()
    capture = LatestFrameCapture(urls)
    capture.start()

    period = 0.0 if FPS_LIMIT <= 0 else 1.0 / FPS_LIMIT
    next_frame_time = time.monotonic()
    last_report_time = time.monotonic()
    frame_count = 0
    delay_sum = 0.0
    last_seq = -1

    try:
        while True:
            if SHOW_CLOCK_WINDOW:
                cv2.imshow(CLOCK_WINDOW, make_clock_image())

            frame, frame_time, frame_seq, backend_name = capture.latest()
            if frame is None or frame_seq == last_seq:
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord("q"):
                    break
                time.sleep(0.002)
                continue
            last_seq = frame_seq

            now = time.monotonic()
            if period > 0 and now < next_frame_time:
                time.sleep(next_frame_time - now)
                now = time.monotonic()
            next_frame_time = now + period

            frame_age_ms = max(0.0, now - frame_time) * 1000.0
            delay_sum += frame_age_ms

            draw_text_panel(frame, [
                f"NOW {now_text()}",
                f"{backend_name}",
                f"program_frame_age={frame_age_ms:.0f} ms",
            ])

            if publisher is not None and node is not None:
                publisher.publish(make_image_msg(frame, node))
                import rclpy

                rclpy.spin_once(node, timeout_sec=0.0)

            if SHOW_PREVIEW:
                cv2.imshow(PREVIEW_WINDOW, frame)
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord("q"):
                    break
                if key == ord("s"):
                    cv2.imwrite("/tmp/rtsp_latency_test.jpg", frame)
                    print("[TEST] 已保存 /tmp/rtsp_latency_test.jpg")

            frame_count += 1
            report_elapsed = time.monotonic() - last_report_time
            if report_elapsed >= 2.0:
                fps = frame_count / report_elapsed
                avg_age = delay_sum / max(1, frame_count)
                print(f"[RTSP] 运行中: {fps:.1f} fps, program_frame_age={avg_age:.0f} ms")
                frame_count = 0
                delay_sum = 0.0
                last_report_time = time.monotonic()

    except KeyboardInterrupt:
        pass
    finally:
        capture.stop()
        cv2.destroyAllWindows()
        stop_ros_node(node)

    return 0


if __name__ == "__main__":
    sys.exit(main())
