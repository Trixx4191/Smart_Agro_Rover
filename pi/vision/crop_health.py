"""
vision/crop_health.py

Captures images from the Pi Camera and analyzes them for
early crop stress indicators using OpenCV HSV color analysis.

Detects:
  - Yellowing (early nitrogen deficiency / disease)
  - Loss of healthy green coverage
  - Stress score: 0.0 (healthy) → 1.0 (severe stress)

Usage:
    from pi.vision.crop_health import CropHealthAnalyzer

    analyzer = CropHealthAnalyzer()
    analyzer.start()

    path, stress_flag, score = analyzer.capture_and_analyze()
    analyzer.stop()
"""

import os
import time
import logging
import numpy as np
from datetime import datetime
from typing import Tuple, Optional

log = logging.getLogger(__name__)


class CropHealthAnalyzer:

    def __init__(self, image_dir: str = None, simulate: bool = False):
        from pi.vision.config import (
            IMAGE_DIR, CAMERA_RESOLUTION, CAMERA_FRAMERATE,
            YELLOW_LOW, YELLOW_HIGH, GREEN_LOW, GREEN_HIGH, STRESS_THRESHOLD
        )
        self.image_dir       = image_dir or IMAGE_DIR
        self.resolution      = CAMERA_RESOLUTION
        self.framerate       = CAMERA_FRAMERATE
        self.yellow_low      = np.array(YELLOW_LOW)
        self.yellow_high     = np.array(YELLOW_HIGH)
        self.green_low       = np.array(GREEN_LOW)
        self.green_high      = np.array(GREEN_HIGH)
        self.stress_threshold = STRESS_THRESHOLD
        self.simulate        = simulate

        self._camera = None
        os.makedirs(self.image_dir, exist_ok=True)

    def start(self):
        if self.simulate:
            log.info("CropHealthAnalyzer: simulation mode")
            return
        try:
            from picamera2 import Picamera2
            self._camera = Picamera2()
            config = self._camera.create_still_configuration(
                main={"size": self.resolution}
            )
            self._camera.configure(config)
            self._camera.start()
            time.sleep(2)   # warm-up
            log.info("CropHealthAnalyzer: camera started")
        except Exception as e:
            log.error(f"CropHealthAnalyzer: camera init failed — {e}")
            log.info("CropHealthAnalyzer: falling back to simulation")
            self.simulate = True

    def stop(self):
        if self._camera:
            try:
                self._camera.stop()
                self._camera.close()
            except Exception:
                pass

    #  Main method 

    def capture_and_analyze(self) -> Tuple[Optional[str], bool, float]:
        """
        Capture an image and analyze it for stress.

        Returns:
            (image_path, stress_flag, stress_score)
            stress_flag  : True if stress detected
            stress_score : 0.0–1.0
        """
        image_path = self._capture_image()
        if image_path is None:
            return None, False, 0.0

        stress_flag, score = self._analyze(image_path)
        return image_path, stress_flag, score

    def analyze_file(self, image_path: str) -> Tuple[bool, float]:
        """Analyze an existing image file."""
        return self._analyze(image_path)

    #  Internal ─

    def _capture_image(self) -> Optional[str]:
        ts        = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename  = f"crop_{ts}.jpg"
        full_path = os.path.join(self.image_dir, filename)

        if self.simulate:
            # Generate a synthetic test image
            return self._generate_test_image(full_path)

        try:
            self._camera.capture_file(full_path)
            log.info(f"CropHealthAnalyzer: captured {filename}")
            return full_path
        except Exception as e:
            log.error(f"CropHealthAnalyzer: capture failed — {e}")
            return None

    def _generate_test_image(self, path: str) -> str:
        """Create a synthetic plant image for testing without real camera."""
        import cv2
        # 640x480 synthetic image: heavily stressed crop (lots of yellow)
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:] = (0, 255, 255)   # fill entire image yellow (BGR) - stressed base

        # Add some healthy green patches (minority)
        cv2.ellipse(img, (100, 400), (60, 40), 60, 0, 360, (0, 180, 0), -1)
        cv2.ellipse(img, (560, 100), (50, 35), 20, 0, 360, (0, 160, 0), -1)
        cv2.ellipse(img, (320, 200), (45, 30), 0,  0, 360, (0, 170, 0), -1)

        cv2.imwrite(path, img)
        log.info(f"CropHealthAnalyzer: synthetic image saved → {path}")
        return path

    def _analyze(self, image_path: str) -> Tuple[bool, float]:
        """
        HSV color space analysis:
          1. Mask yellow pixels → yellowing stress indicator
          2. Mask green pixels  → healthy tissue
          3. Stress score = yellow_pixels / (yellow + green pixels)
             If score > threshold → flag stress
        """
        try:
            import cv2
        except ImportError:
            log.error("OpenCV not installed — pip install opencv-python")
            return False, 0.0

        try:
            img = cv2.imread(image_path)
            if img is None:
                log.error(f"CropHealthAnalyzer: could not read {image_path}")
                return False, 0.0

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Masks
            yellow_mask = cv2.inRange(hsv, self.yellow_low, self.yellow_high)
            green_mask  = cv2.inRange(hsv, self.green_low,  self.green_high)

            yellow_px = int(np.sum(yellow_mask > 0))
            green_px  = int(np.sum(green_mask  > 0))
            plant_px  = yellow_px + green_px

            if plant_px == 0:
                log.debug("CropHealthAnalyzer: no plant pixels detected")
                return False, 0.0

            score = yellow_px / plant_px
            stress = score > self.stress_threshold

            log.info(f"CropHealthAnalyzer: yellow={yellow_px}px "
                     f"green={green_px}px score={score:.3f} "
                     f"stress={'YES' if stress else 'no'}")

            # Save annotated debug image
            self._save_annotated(img, yellow_mask, green_mask, score, image_path)

            return stress, round(score, 4)

        except Exception as e:
            log.error(f"CropHealthAnalyzer: analysis error — {e}")
            return False, 0.0

    def _save_annotated(self, img, yellow_mask, green_mask, score, orig_path):
        """Save a side-by-side annotated version for the dashboard."""
        try:
            import cv2
            annotated = img.copy()

            # Overlay yellow regions in red, green in bright green
            annotated[yellow_mask > 0] = (0, 0, 200)   # red in BGR
            annotated[green_mask  > 0] = (0, 200, 0)   # bright green

            # Label
            label = f"Stress: {score:.1%}"
            color = (0, 0, 200) if score > self.stress_threshold else (0, 180, 0)
            cv2.putText(annotated, label, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

            annotated_path = orig_path.replace(".jpg", "_annotated.jpg")
            cv2.imwrite(annotated_path, annotated)
        except Exception:
            pass   # annotation is non-critical
