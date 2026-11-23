"""다른 PC에서 clone만 받아도 실제 차량 모델과 웹 소스가 제공되는지 검증한다."""
from hashlib import sha256
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import unittest
from unittest.mock import patch
from urllib.request import urlopen

import cv2
import numpy as np

from scripts.http_map_server import BUNDLED_WEB_ROOT, handler_for, vehicle_model_path


class BundledWebTest(unittest.TestCase):
    def test_real_single_class_model_matches_manifest_and_runs(self):
        model = vehicle_model_path(BUNDLED_WEB_ROOT)
        manifest = json.loads((model.parent / 'manifest.json').read_text(encoding='utf-8'))
        self.assertEqual(model.stat().st_size, manifest['size_bytes'])
        self.assertEqual(sha256(model.read_bytes()).hexdigest(), manifest['sha256'])
        self.assertEqual(manifest['labels'], ['car'])
        net = cv2.dnn.readNetFromONNX(str(model))
        net.setInput(np.zeros(manifest['input_shape'], dtype=np.float32))
        output = net.forward()
        self.assertEqual(list(output.shape), manifest['output_shape'])
        self.assertTrue(np.isfinite(output).all())

    def test_default_web_and_exact_model_are_served_over_http(self):
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            handler = handler_for(BUNDLED_WEB_ROOT, root / 'maps', root / 'labels.json', root / 'mapping.json')
            server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                base = f'http://127.0.0.1:{server.server_port}'
                for path in ('/', '/js/app.js', '/js/MediaHandler.js', '/js/OverlayRenderer.js',
                             '/js/StorageManager.js', '/js/ArUcoBridge.js', '/js/YOLODetector.js',
                             '/rover-original-yolo.js', '/parking-detection-worker.js'):
                    with urlopen(base + path) as response:
                        body = response.read()
                    self.assertTrue(body, path)
                with urlopen(base + '/models/parking_cars.onnx') as response:
                    served = response.read()
                self.assertEqual(sha256(served).hexdigest(),
                                 sha256(vehicle_model_path(BUNDLED_WEB_ROOT).read_bytes()).hexdigest())
            finally:
                server.shutdown()
                thread.join()
                server.server_close()

    def test_external_web_without_model_uses_bundled_weights(self):
        with TemporaryDirectory() as temporary:
            self.assertEqual(vehicle_model_path(Path(temporary)), BUNDLED_WEB_ROOT / 'models/parking_cars.onnx')
            with patch('scripts.http_map_server.BUNDLED_WEB_ROOT', Path(temporary)):
                with self.assertRaises(FileNotFoundError):
                    vehicle_model_path(Path(temporary))


if __name__ == '__main__':
    unittest.main()
