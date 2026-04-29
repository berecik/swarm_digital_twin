"""
Pillow-based domain-randomisation augmentations.

The full data pipeline (deferred) uses albumentations for GPU-friendly
augmentations. This module ships the CI-deliverable subset built on
Pillow alone so unit tests can verify the augmentation surface without
adding heavy dependencies. Each `Augmentation` is deterministic when
seeded so the regression suite stays reproducible.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from PIL import Image, ImageEnhance, ImageFilter


@dataclass(frozen=True)
class AugmentationSpec:
    """Tunable parameters for the available augmentations."""
    brightness_low: float = 0.6
    brightness_high: float = 1.4
    contrast_low: float = 0.7
    contrast_high: float = 1.3
    blur_radius_max: float = 1.5
    cutout_count_max: int = 3
    cutout_size_frac_max: float = 0.15
    weather_drop_density: float = 0.002   # rain/snow density (frac of pixels)


class Augmenter:
    """Apply seeded domain-randomisation augmentations to PIL images."""

    def __init__(self, spec: Optional[AugmentationSpec] = None,
                 seed: Optional[int] = None) -> None:
        self.spec = spec or AugmentationSpec()
        self._rng = np.random.default_rng(seed)

    def random_brightness(self, img: Image.Image) -> Image.Image:
        factor = float(self._rng.uniform(
            self.spec.brightness_low, self.spec.brightness_high))
        return ImageEnhance.Brightness(img).enhance(factor)

    def random_contrast(self, img: Image.Image) -> Image.Image:
        factor = float(self._rng.uniform(
            self.spec.contrast_low, self.spec.contrast_high))
        return ImageEnhance.Contrast(img).enhance(factor)

    def random_blur(self, img: Image.Image) -> Image.Image:
        radius = float(self._rng.uniform(0.0, self.spec.blur_radius_max))
        return img.filter(ImageFilter.GaussianBlur(radius=radius))

    def random_cutout(self, img: Image.Image) -> Image.Image:
        """Black out N random rectangular patches (regularises the model)."""
        out = img.copy()
        arr = np.asarray(out).copy()
        h, w = arr.shape[:2]
        n = int(self._rng.integers(0, self.spec.cutout_count_max + 1))
        for _ in range(n):
            ch = int(self._rng.uniform(
                0.05, self.spec.cutout_size_frac_max) * h)
            cw = int(self._rng.uniform(
                0.05, self.spec.cutout_size_frac_max) * w)
            cy = int(self._rng.integers(0, max(1, h - ch)))
            cx = int(self._rng.integers(0, max(1, w - cw)))
            arr[cy:cy + ch, cx:cx + cw] = 0
        return Image.fromarray(arr)

    def add_weather(self, img: Image.Image, kind: str = "rain") -> Image.Image:
        """Sprinkle rain (white streaks) or snow (white dots) on the frame."""
        if kind not in {"rain", "snow"}:
            raise ValueError(f"unknown weather kind '{kind}'; "
                             f"expected 'rain' or 'snow'")
        out = img.copy()
        arr = np.asarray(out).copy()
        h, w = arr.shape[:2]
        n_drops = int(h * w * self.spec.weather_drop_density)
        ys = self._rng.integers(0, h, size=n_drops)
        xs = self._rng.integers(0, w, size=n_drops)
        if kind == "rain":
            # Vertical streaks 3 px tall.
            for dy in range(3):
                ys2 = np.clip(ys + dy, 0, h - 1)
                arr[ys2, xs] = 255
        else:
            arr[ys, xs] = 255
        return Image.fromarray(arr)

    def apply_random_chain(self, img: Image.Image,
                           p: float = 0.5) -> Image.Image:
        """Apply each augmentation with probability *p* in a fixed order."""
        out = img
        if self._rng.random() < p:
            out = self.random_brightness(out)
        if self._rng.random() < p:
            out = self.random_contrast(out)
        if self._rng.random() < p:
            out = self.random_blur(out)
        if self._rng.random() < p:
            out = self.random_cutout(out)
        if self._rng.random() < p:
            out = self.add_weather(
                out, kind="rain" if self._rng.random() < 0.5 else "snow")
        return out
