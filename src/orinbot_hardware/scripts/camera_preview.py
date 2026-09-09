"""Bounded JPEG preview shared by the mapping and guide HTTP servers."""
import io
import threading
import time

from PIL import Image


def image_jpeg(message):
    formats = {'rgb8': ('RGB', 'RGB', 3), 'bgr8': ('RGB', 'BGR', 3),
               'rgba8': ('RGBA', 'RGBA', 4), 'bgra8': ('RGBA', 'BGRA', 4),
               'mono8': ('L', 'L', 1)}
    if message.encoding not in formats:
        raise ValueError('지원하지 않는 카메라 영상 형식: ' + message.encoding)
    mode, raw, channels = formats[message.encoding]
    width, height, step = message.width, message.height, message.step
    if not (0 < width <= 4096 and 0 < height <= 2160 and width * channels <= step <= 32768):
        raise ValueError('잘못된 카메라 영상 크기입니다.')
    if len(message.data) != step * height:
        raise ValueError('카메라 영상 데이터 길이가 맞지 않습니다.')
    image = Image.frombytes(mode, (width, height), bytes(message.data), 'raw', raw, step, 1)
    image.thumbnail((640, 480))
    buffer = io.BytesIO()
    image.convert('RGB').save(buffer, format='JPEG', quality=70)
    return buffer.getvalue()


class CameraPreview:
    def __init__(self, node):
        from sensor_msgs.msg import Image as ROSImage
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.node = node
        self.lock = threading.Lock()
        self.jpeg = None
        self.received = self.stamp = 0.
        self.requested = -float('inf')
        self.encoded = -float('inf')
        self.error = '카메라 영상을 기다리는 중입니다. 카메라 실행 상태를 확인하세요.'
        # Keep image conversion out of the navigation callback group; queue only
        # the latest frame and encode only while a browser is requesting images.
        self.group = MutuallyExclusiveCallbackGroup()
        self.subscription = node.create_subscription(
            ROSImage, '/camera/color/image_raw', self.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.group)

    def receive(self, message):
        now = time.monotonic()
        with self.lock:
            if now - self.requested > 2 or now - self.encoded < .2:
                return
            self.encoded = now
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        age = self.node.get_clock().now().nanoseconds / 1e9 - stamp
        try:
            if not -.1 <= age <= 1:
                raise ValueError('카메라 영상의 시간이 오래되었거나 맞지 않습니다.')
            jpeg = image_jpeg(message)
        except (ValueError, OSError) as error:
            with self.lock:
                self.jpeg = None
                self.error = str(error)
            return
        with self.lock:
            self.jpeg, self.received, self.stamp, self.error = jpeg, now, stamp, ''

    def response(self):
        now = time.monotonic()
        with self.lock:
            self.requested = now
            age = self.node.get_clock().now().nanoseconds / 1e9 - self.stamp
            if self.jpeg is not None and now - self.received <= 1 and -.1 <= age <= 1:
                return 200, self.jpeg, 'image/jpeg'
            message = self.error or '카메라 영상 수신이 끊겼습니다. 연결 상태를 확인하세요.'
            import json
            return 503, json.dumps({'message': message}, ensure_ascii=False).encode(), 'application/json'


def serve_camera(handler, node, path):
    if path == '/camera_preview.js':
        handler.reply(200, (node.share / 'config/camera_preview.js').read_bytes(), 'text/javascript; charset=utf-8')
        return True
    if path == '/api/camera.jpg':
        handler.reply(*node.camera_preview.response())
        return True
    return False
