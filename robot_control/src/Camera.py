"""
控制层相机接口。

这个文件不直接连接 RTSP，也不关心仿真/实物相机细节；它只订阅底层统一
图像话题 `/robot/image_raw`，给控制逻辑提供当前帧。
"""

import os
import time
from dataclasses import dataclass
from threading import Lock
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


@dataclass
class CameraConfig:
    image_topic: str = "/robot/image_raw"
    node_name: str = "camera_reader"


class Camera(Node):
    """控制层读取图像的简单接口。"""

    def __init__(self, image_topic: str = "/robot/image_raw", node_name: str = "camera_reader"):
        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True
        else:
            self._owns_rclpy = False

        self.config = CameraConfig(image_topic=image_topic, node_name=node_name)
        super().__init__(self.config.node_name)

        self._lock = Lock()
        self._frame: Optional[np.ndarray] = None
        self._last_frame_time: Optional[float] = None

        self._image_sub = self.create_subscription(
            Image,
            self.config.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"Camera已启动: image={self.config.image_topic}")

    # ------------------------------------------------------------------
    # 外部接口
    # ------------------------------------------------------------------

    def Read(self, timeout_s: float = 0.0) -> Optional[np.ndarray]:
        """
        读取当前帧。

        参数：
            timeout_s：如果当前还没有图像，最多等待多少秒。

        返回：
            OpenCV BGR 图像；如果没有收到图像则返回 None。
        """
        self._spin_for_frame(timeout_s)
        with self._lock:
            if self._frame is None:
                return None
            return self._frame.copy()

    def Show(
        self,
        window_name: str = "camera",
        wait_ms: int = 1,
        timeout_s: float = 0.0,
    ) -> bool:
        """
        把当前帧显示在指定窗口。

        返回：
            True：显示成功。
            False：当前没有图像。
        """
        frame = self.Read(timeout_s=timeout_s)
        if frame is None:
            self.get_logger().warn("当前还没有收到图像，无法显示")
            return False

        cv2.imshow(window_name, frame)
        cv2.waitKey(wait_ms)
        return True

    def Save(self, path: str, timeout_s: float = 0.0) -> bool:
        """
        把当前帧保存到指定路径。

        返回：
            True：保存成功。
            False：当前没有图像或写入失败。
        """
        frame = self.Read(timeout_s=timeout_s)
        if frame is None:
            self.get_logger().warn("当前还没有收到图像，无法保存")
            return False

        directory = os.path.dirname(os.path.abspath(path))
        if directory:
            os.makedirs(directory, exist_ok=True)

        ok = cv2.imwrite(path, frame)
        if not ok:
            self.get_logger().error(f"图像保存失败: {path}")
            return False

        self.get_logger().info(f"图像已保存: {path}")
        return True


    def last_frame_age(self) -> Optional[float]:
        """返回当前缓存图像的年龄，单位秒；没有图像时返回 None。"""
        with self._lock:
            if self._last_frame_time is None:
                return None
            return time.monotonic() - self._last_frame_time

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 内部实现
    # ------------------------------------------------------------------

    def _spin_for_frame(self, timeout_s: float) -> None:
        start_time = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            with self._lock:
                has_frame = self._frame is not None
            if has_frame or timeout_s <= 0.0:
                return
            if time.monotonic() - start_time >= timeout_s:
                return
            time.sleep(0.005)

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self._image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return

        with self._lock:
            self._frame = frame
            self._last_frame_time = time.monotonic()

    @staticmethod
    def _image_to_bgr(msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if encoding in ("bgr8", "rgb8", "8uc3"):
            row_pixels = msg.step // 3
            image = data.reshape((msg.height, row_pixels, 3))[:, :msg.width, :]
            if encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            return image.copy()

        if encoding in ("bgra8", "rgba8", "8uc4"):
            row_pixels = msg.step // 4
            image = data.reshape((msg.height, row_pixels, 4))[:, :msg.width, :]
            if encoding == "rgba8":
                image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            else:
                image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
            return image.copy()

        if encoding in ("mono8", "8uc1"):
            image = data.reshape((msg.height, msg.step))[:, :msg.width]
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        raise ValueError(f"暂不支持的图像编码: {msg.encoding}")


if __name__ == "__main__":
    camera = Camera()
    window_name = "camera"
    last_notice_time = 0.0
    try:
        while rclpy.ok():
            frame = camera.Read(timeout_s=0.0)
            if frame is None:
                now = time.monotonic()
                if now - last_notice_time >= 2.0:
                    camera.get_logger().warn(
                        f"还没有收到 {camera.config.image_topic}，请先启动底层相机发布节点"
                    )
                    last_notice_time = now

                placeholder = np.zeros((260, 760, 3), dtype=np.uint8)
                cv2.putText(
                    placeholder,
                    f"waiting for {camera.config.image_topic}",
                    (35, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    placeholder,
                    "press q or Esc to quit",
                    (35, 175),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (180, 180, 180),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow(window_name, placeholder)
            else:
                cv2.imshow(window_name, frame)

            key = cv2.waitKey(30) & 0xFF
            if key == 27 or key == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        camera.destroy_node()
