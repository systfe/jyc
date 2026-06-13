#!/usr/bin/env python3

import os
import sys
import threading
import time
from typing import Iterable, List, Optional, Tuple
from urllib.parse import quote, urlparse

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
except Exception:
    Gst = None


class LatestFrameCapture:
    """持续读取相机流，并且只保留最新解码出来的一帧。"""

    def __init__(self, node: Node):
        self.node = node
        self.urls = self._candidate_urls()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._latest_frame = None
        self._latest_time = 0.0
        self._latest_seq = 0
        self._cap = None
        self._gst_pipeline = None
        self._gst_sink = None

    def start(self) -> bool:
        if not self.urls:
            self.node.get_logger().error(
                "没有可用的相机候选地址，请检查网络或 rtsp_url/camera_ip"
            )
            return False
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self._close_gstreamer()

    def latest(self) -> Tuple[Optional[object], float, int]:
        with self._lock:
            if self._latest_frame is None:
                return None, 0.0, self._latest_seq
            return self._latest_frame.copy(), self._latest_time, self._latest_seq

    def _candidate_urls(self) -> List[str]:
        rtsp_url = self.node.get_parameter("rtsp_url").value
        if rtsp_url:
            return [rtsp_url]

        urls = []
        for ip in self._candidate_ips():
            for path in self.node.get_parameter("http_paths").value:
                urls.append(self._build_http_url(ip, path))
            for path in self.node.get_parameter("rtsp_paths").value:
                urls.append(self._build_rtsp_url(ip, path))
        return urls

    def _candidate_ips(self) -> List[str]:
        camera_ip = self.node.get_parameter("camera_ip").value
        if camera_ip:
            return [camera_ip]

        self.node.get_logger().error("camera_ip 为空，并且 rtsp_url 也没有填写")
        return []

    def _build_rtsp_url(self, ip: str, path: str) -> str:
        if path and not path.startswith("/"):
            path = "/" + path

        username = self.node.get_parameter("username").value
        password = self.node.get_parameter("password").value
        rtsp_port = self.node.get_parameter("rtsp_port").value

        auth = ""
        if username:
            auth = quote(username, safe="")
            if password:
                auth += ":" + quote(password, safe="")
            auth += "@"

        return f"rtsp://{auth}{ip}:{rtsp_port}{path}"

    def _build_http_url(self, ip: str, path: str) -> str:
        if path and not path.startswith("/"):
            path = "/" + path

        username = self.node.get_parameter("username").value
        password = self.node.get_parameter("password").value

        auth = ""
        if username:
            auth = quote(username, safe="")
            if password:
                auth += ":" + quote(password, safe="")
            auth += "@"

        return f"http://{auth}{ip}{path}"

    def _ffmpeg_profile_options(self, profile_name: str) -> dict:
        open_timeout_us = str(self.node.get_parameter("open_timeout_ms").value * 1000)
        profiles = {
            "http_plain": {},
            "plain": {},
            "udp_low_delay": {
                "rtsp_transport": "udp",
                "fflags": "nobuffer",
                "flags": "low_delay",
                "max_delay": "0",
                "reorder_queue_size": "0",
                "stimeout": open_timeout_us,
            },
            "tcp_low_delay": {
                "rtsp_transport": "tcp",
                "fflags": "nobuffer",
                "flags": "low_delay",
                "max_delay": "0",
                "reorder_queue_size": "0",
                "stimeout": open_timeout_us,
            },
            "tcp_compatible": {
                "rtsp_transport": "tcp",
                "stimeout": "5000000",
                "max_delay": "500000",
            },
        }
        return profiles.get(profile_name, profiles["tcp_compatible"])

    def _configure_ffmpeg_profile(self, profile_name: str) -> None:
        options = self._ffmpeg_profile_options(profile_name)
        if options:
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "|".join(
                f"{key};{value}" for key, value in options.items()
            )
        else:
            os.environ.pop("OPENCV_FFMPEG_CAPTURE_OPTIONS", None)

    def _make_capture(self, url: str) -> cv2.VideoCapture:
        use_open_params = self.node.get_parameter("use_open_params").value
        if not use_open_params:
            return cv2.VideoCapture(url, cv2.CAP_FFMPEG)

        params = []
        if self.node.get_parameter("use_hw_acceleration").value and hasattr(cv2, "CAP_PROP_HW_ACCELERATION"):
            params.extend([cv2.CAP_PROP_HW_ACCELERATION, cv2.VIDEO_ACCELERATION_ANY])
        if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
            params.extend([cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, self.node.get_parameter("open_timeout_ms").value])
        if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
            params.extend([cv2.CAP_PROP_READ_TIMEOUT_MSEC, self.node.get_parameter("read_timeout_ms").value])

        if params:
            try:
                return cv2.VideoCapture(url, cv2.CAP_FFMPEG, params)
            except Exception:
                pass
        return cv2.VideoCapture(url, cv2.CAP_FFMPEG)

    def _open_capture(self, urls: Iterable[str]) -> Optional[cv2.VideoCapture]:
        capture_width = self.node.get_parameter("capture_width").value
        capture_height = self.node.get_parameter("capture_height").value

        for url in urls:
            scheme = urlparse(url).scheme.lower()
            if scheme == "http":
                profiles = self.node.get_parameter("http_open_profiles").value
            else:
                profiles = self.node.get_parameter("rtsp_open_profiles").value

            for profile_name in profiles:
                self._configure_ffmpeg_profile(profile_name)
                self.node.get_logger().info(f"正在尝试相机流 ({profile_name}): {url}")
                cap = self._make_capture(url)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if capture_width > 0:
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, capture_width)
                if capture_height > 0:
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_height)

                if not cap.isOpened():
                    cap.release()
                    continue

                ok, frame = cap.read()
                if ok and frame is not None:
                    self.node.get_logger().info(
                        f"已打开相机流: {url}, profile={profile_name}, "
                        f"size={frame.shape[1]}x{frame.shape[0]}"
                    )
                    return cap

                cap.release()

        return None

    def _open_gstreamer(self, urls: Iterable[str]) -> bool:
        if Gst is None:
            self.node.get_logger().warn("GStreamer Python 绑定不可用")
            return False

        Gst.init(None)
        username = self.node.get_parameter("username").value
        password = self.node.get_parameter("password").value
        latency_ms = self.node.get_parameter("gst_latency_ms").value
        protocols = self.node.get_parameter("gst_protocols").value

        for url in urls:
            if urlparse(url).scheme.lower() != "rtsp":
                continue

            auth_free_url = url
            if username:
                auth_prefix = f"{quote(username, safe='')}:"
                if password:
                    auth_prefix += f"{quote(password, safe='')}@"
                if auth_prefix in auth_free_url:
                    auth_free_url = auth_free_url.replace(auth_prefix, "")

            pipeline = (
                f"rtspsrc location=\"{auth_free_url}\" "
                f"user-id=\"{username}\" user-pw=\"{password}\" "
                f"protocols={protocols} latency={latency_ms} "
                "drop-on-latency=true do-retransmission=false ! "
                "rtph264depay ! decodebin ! videoconvert ! "
                "video/x-raw,format=BGR ! "
                "appsink name=sink emit-signals=false max-buffers=1 drop=true sync=false"
            )

            self.node.get_logger().info(f"正在尝试 RTSP (gstreamer): {url}")
            try:
                self._gst_pipeline = Gst.parse_launch(pipeline)
                self._gst_sink = self._gst_pipeline.get_by_name("sink")
                self._gst_pipeline.set_state(Gst.State.PLAYING)
                deadline = time.monotonic() + self.node.get_parameter("open_timeout_ms").value / 1000.0
                while time.monotonic() < deadline and not self._stop_event.is_set():
                    sample = self._gst_sink.emit("try-pull-sample", 200 * Gst.MSECOND)
                    if sample is not None:
                        frame = self._frame_from_gst_sample(sample)
                        if frame is not None:
                            with self._lock:
                                self._latest_frame = frame
                                self._latest_time = time.monotonic()
                                self._latest_seq += 1
                            self.node.get_logger().info(
                                f"已用 GStreamer 打开 RTSP: {url}, "
                                f"size={frame.shape[1]}x{frame.shape[0]}"
                            )
                            return True

                self._close_gstreamer()
            except Exception as exc:
                self.node.get_logger().warn(f"GStreamer 打开失败: {exc}")
                self._close_gstreamer()

        return False

    def _close_gstreamer(self) -> None:
        if self._gst_pipeline is not None and Gst is not None:
            self._gst_pipeline.set_state(Gst.State.NULL)
        self._gst_pipeline = None
        self._gst_sink = None

    def _frame_from_gst_sample(self, sample):
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
            import numpy as np

            frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape((height, width, 3)).copy()
            return self._resize_frame(frame)
        finally:
            buf.unmap(map_info)

    def _read_gstreamer_frame(self) -> bool:
        if self._gst_sink is None or Gst is None:
            return False

        sample = self._gst_sink.emit("try-pull-sample", 50 * Gst.MSECOND)
        if sample is None:
            return True

        frame = self._frame_from_gst_sample(sample)
        if frame is None:
            return True

        with self._lock:
            self._latest_frame = frame
            self._latest_time = time.monotonic()
            self._latest_seq += 1
        return True

    def _resize_frame(self, frame):
        output_width = self.node.get_parameter("output_width").value
        output_height = self.node.get_parameter("output_height").value
        if output_width <= 0 and output_height <= 0:
            return frame

        if output_width > 0 and output_height > 0:
            size = (output_width, output_height)
        elif output_width > 0:
            scale = output_width / frame.shape[1]
            size = (output_width, max(1, int(frame.shape[0] * scale)))
        else:
            scale = output_height / frame.shape[0]
            size = (max(1, int(frame.shape[1] * scale)), output_height)

        return cv2.resize(frame, size, interpolation=cv2.INTER_AREA)

    def _run(self) -> None:
        reconnect_delay_s = self.node.get_parameter("reconnect_delay_s").value
        backend = self.node.get_parameter("backend").value
        while not self._stop_event.is_set():
            if backend in ("gstreamer", "auto") and self._open_gstreamer(self.urls):
                while not self._stop_event.is_set():
                    if not self._read_gstreamer_frame():
                        break
                self._close_gstreamer()
                time.sleep(reconnect_delay_s)
                continue

            if backend == "gstreamer":
                time.sleep(reconnect_delay_s)
                continue

            self._cap = self._open_capture(self.urls)
            if self._cap is None:
                time.sleep(reconnect_delay_s)
                continue

            while not self._stop_event.is_set():
                ok, frame = self._cap.read()
                if not ok or frame is None:
                    self.node.get_logger().warn("RTSP 读帧失败，正在重连")
                    self._cap.release()
                    self._cap = None
                    time.sleep(reconnect_delay_s)
                    break

                frame = self._resize_frame(frame)
                with self._lock:
                    self._latest_frame = frame
                    self._latest_time = time.monotonic()
                    self._latest_seq += 1


class RtspCameraPublisher(Node):
    def __init__(self):
        super().__init__("rtsp_camera_publisher")
        self._declare_parameters()
        self._configure_ffmpeg()

        topic = self.get_parameter("image_topic").value
        queue_size = self.get_parameter("publish_queue_size").value
        image_qos = QoSProfile(depth=max(1, int(queue_size)))
        image_qos.history = QoSHistoryPolicy.KEEP_LAST
        image_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.publisher = self.create_publisher(Image, topic, image_qos)

        self.capture = LatestFrameCapture(self)
        if not self.capture.start():
            raise RuntimeError("RTSP 采集启动失败")

        fps_limit = self.get_parameter("fps_limit").value
        period = 0.001 if fps_limit <= 0 else 1.0 / fps_limit
        self.timer = self.create_timer(period, self._publish_latest_frame)

        self.frame_count = 0
        self.delay_sum = 0.0
        self.last_seq = -1
        self.last_report_time = time.monotonic()
        self.get_logger().info(f"正在发布 RTSP 图像到 {topic}")

    def destroy_node(self):
        if hasattr(self, "capture"):
            self.capture.stop()
        super().destroy_node()

    def _declare_parameters(self) -> None:
        self.declare_parameter("camera_ip", "192.168.10.100")
        self.declare_parameter("rtsp_port", 554)
        self.declare_parameter("username", "admin")
        self.declare_parameter("password", "zxcvbnm12")
        self.declare_parameter("rtsp_url", "")
        self.declare_parameter("backend", "opencv")
        self.declare_parameter("gst_latency_ms", 0)
        self.declare_parameter("gst_protocols", "udp")
        self.declare_parameter("http_open_profiles", [
            "http_plain",
        ])
        self.declare_parameter("rtsp_open_profiles", [
            "udp_low_delay",
            "tcp_low_delay",
            "tcp_compatible",
            "plain",
        ])
        self.declare_parameter("http_paths", [
            "/video",
            "/mjpeg",
            "/mjpg/video.mjpg",
            "/videostream.cgi",
            "/cgi-bin/mjpg/video.cgi?channel=1&subtype=1",
            "/ISAPI/Streaming/channels/102/httpPreview",
        ])
        self.declare_parameter("rtsp_paths", [
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
        ])
        self.declare_parameter("rtsp_transport", "tcp")
        self.declare_parameter("open_timeout_ms", 2000)
        self.declare_parameter("read_timeout_ms", 500)
        self.declare_parameter("use_open_params", False)
        self.declare_parameter("use_hw_acceleration", False)
        self.declare_parameter("image_topic", "/robot/image_raw")
        self.declare_parameter("frame_id", "fisheye_camera")
        self.declare_parameter("fps_limit", 30.0)
        self.declare_parameter("capture_width", 0)
        self.declare_parameter("capture_height", 0)
        self.declare_parameter("output_width", 640)
        self.declare_parameter("output_height", 640)
        self.declare_parameter("reconnect_delay_s", 0.5)
        self.declare_parameter("publish_queue_size", 1)
        self.declare_parameter("log_interval_s", 2.0)

    def _configure_ffmpeg(self) -> None:
        transport = self.get_parameter("rtsp_transport").value
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            f"rtsp_transport;{transport}|stimeout;5000000|max_delay;500000"
        )
        cv2.setNumThreads(1)

    def _publish_latest_frame(self) -> None:
        frame, frame_time, frame_seq = self.capture.latest()
        if frame is None:
            return
        if frame_seq == self.last_seq:
            return
        self.last_seq = frame_seq

        self.publisher.publish(self._make_image_msg(frame))

        now = time.monotonic()
        self.frame_count += 1
        self.delay_sum += max(0.0, now - frame_time)
        log_interval_s = self.get_parameter("log_interval_s").value
        if now - self.last_report_time >= log_interval_s:
            fps = self.frame_count / max(1e-6, now - self.last_report_time)
            frame_age_ms = 1000.0 * self.delay_sum / max(1, self.frame_count)
            self.get_logger().info(f"运行中: {fps:.1f} fps, frame_age={frame_age_ms:.0f} ms")
            self.frame_count = 0
            self.delay_sum = 0.0
            self.last_report_time = now

    def _make_image_msg(self, frame) -> Image:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        return msg


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = RtspCameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"rtsp_camera_publisher 运行失败: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
