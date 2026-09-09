import io
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from camera_preview import CameraPreview, image_jpeg


def frame(encoding='rgb8', data=None, width=2, height=2, step=6, stamp=100):
    return SimpleNamespace(encoding=encoding, width=width, height=height, step=step,
                           data=bytes([255, 0, 0] * 4) if data is None else data,
                           header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nanosec=0)))


class CameraPreviewTests(unittest.TestCase):
    def test_rgb_and_padded_bgr_color_order(self):
        for message in (frame(), frame('bgr8', bytes([0,0,255,0,0,255,0,0]*2), step=8)):
            image = Image.open(io.BytesIO(image_jpeg(message)))
            self.assertEqual(image.size, (2, 2))
            r, g, b = image.getpixel((0, 0))
            self.assertGreater(r, 240)
            self.assertLess(g, 10)
            self.assertLess(b, 10)

    def test_grayscale_and_resolution_bound(self):
        image = Image.open(io.BytesIO(image_jpeg(frame('mono8', bytes([100])*1280*720,1280,720,1280))))
        self.assertEqual(image.size, (640, 360))
        self.assertEqual(image.mode, 'RGB')

    def test_bad_frames_rejected(self):
        for message in (frame('16UC1'), frame(step=1), frame(data=b'x'), frame(width=100000)):
            with self.assertRaises(ValueError): image_jpeg(message)

    def preview(self):
        self.now = 100
        clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(self.now*1e9)))
        node = SimpleNamespace(get_clock=lambda: clock, create_subscription=lambda *a, **kw: None)
        return CameraPreview(node)

    def test_no_viewer_skips_encoding_and_old_frames_are_not_served(self):
        preview = self.preview()
        with patch('camera_preview.time.monotonic', return_value=100):
            preview.receive(frame())
            self.assertIsNone(preview.jpeg)
            self.assertEqual(preview.response()[0], 503)
            preview.receive(frame())
            self.assertEqual(preview.response()[0], 200)
        self.now = 102
        with patch('camera_preview.time.monotonic', return_value=102):
            self.assertEqual(preview.response()[0], 503)
            preview.receive(frame(stamp=100))
            self.assertIsNone(preview.jpeg)
            self.assertEqual(preview.response()[0], 503)
        with patch('camera_preview.time.monotonic', return_value=102.3):
            preview.receive(frame(stamp=102))
            self.assertEqual(preview.response()[0], 200)

    def test_encoding_rate_and_malformed_frame_invalidate_cache(self):
        preview = self.preview()
        with patch('camera_preview.time.monotonic', return_value=100):
            preview.response(); preview.receive(frame())
        with patch('camera_preview.time.monotonic', return_value=100.1), patch('camera_preview.image_jpeg') as encode:
            preview.receive(frame()); encode.assert_not_called()
        with patch('camera_preview.time.monotonic', return_value=100.3):
            preview.receive(frame(data=b'x'))
            self.assertEqual(preview.response()[0], 503)


if __name__ == '__main__':
    unittest.main()
