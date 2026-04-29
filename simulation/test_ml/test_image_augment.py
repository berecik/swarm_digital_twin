"""Tests for the Pillow-based image augmentation."""

import numpy as np
import pytest
from PIL import Image

from ml.image_augment import AugmentationSpec, Augmenter


def _img(rgb=(120, 130, 140), size=(64, 48)):
    return Image.new("RGB", size, rgb)


class TestAugmenter:
    def test_seeded_brightness_is_reproducible(self):
        a = Augmenter(seed=42)
        b = Augmenter(seed=42)
        out_a = a.random_brightness(_img())
        out_b = b.random_brightness(_img())
        np.testing.assert_array_equal(np.asarray(out_a), np.asarray(out_b))

    def test_seeded_chain_is_reproducible(self):
        a = Augmenter(seed=99)
        b = Augmenter(seed=99)
        out_a = a.apply_random_chain(_img(), p=1.0)
        out_b = b.apply_random_chain(_img(), p=1.0)
        np.testing.assert_array_equal(np.asarray(out_a), np.asarray(out_b))

    def test_random_brightness_changes_image(self):
        a = Augmenter(seed=1)
        original = _img()
        out = a.random_brightness(original)
        # Different brightness factor should produce different pixels
        # (with seed=1, factor != 1.0 with overwhelming probability).
        assert not np.array_equal(np.asarray(original), np.asarray(out))

    def test_random_blur_preserves_size(self):
        out = Augmenter(seed=2).random_blur(_img(size=(80, 60)))
        assert out.size == (80, 60)

    def test_random_cutout_blacks_some_pixels(self):
        a = Augmenter(seed=3,
                      spec=AugmentationSpec(cutout_count_max=5,
                                             cutout_size_frac_max=0.3))
        out = a.random_cutout(_img(rgb=(255, 255, 255)))
        # At least one black pixel must appear (cutouts are zero-fill).
        arr = np.asarray(out)
        assert (arr == 0).any()

    def test_add_weather_rain(self):
        out = Augmenter(seed=4).add_weather(_img(rgb=(0, 0, 0)), kind="rain")
        # Rain adds white pixels.
        assert (np.asarray(out) == 255).any()

    def test_add_weather_unknown_kind_raises(self):
        with pytest.raises(ValueError, match="unknown weather kind"):
            Augmenter(seed=5).add_weather(_img(), kind="hail")

    def test_chain_with_p_zero_is_identity(self):
        original = _img()
        out = Augmenter(seed=6).apply_random_chain(original, p=0.0)
        np.testing.assert_array_equal(np.asarray(out), np.asarray(original))

    def test_random_contrast_changes_image(self):
        original = _img()
        out = Augmenter(seed=7).random_contrast(original)
        assert not np.array_equal(np.asarray(original), np.asarray(out))

    def test_random_contrast_preserves_size(self):
        out = Augmenter(seed=8).random_contrast(_img(size=(72, 56)))
        assert out.size == (72, 56)

    def test_add_weather_snow(self):
        out = Augmenter(seed=9).add_weather(_img(rgb=(0, 0, 0)), kind="snow")
        # Snow scatters white dots (no streaks).
        arr = np.asarray(out)
        assert (arr == 255).any()

    def test_default_spec_values(self):
        spec = AugmentationSpec()
        assert 0.0 < spec.brightness_low < 1.0 < spec.brightness_high
        assert 0.0 < spec.contrast_low < 1.0 < spec.contrast_high
        assert spec.blur_radius_max > 0
        assert spec.cutout_count_max >= 1
        assert 0.0 < spec.cutout_size_frac_max <= 1.0
        assert spec.weather_drop_density > 0

    def test_unseeded_augmenter_runs(self):
        # Should not crash when no seed is provided.
        out = Augmenter().apply_random_chain(_img(), p=1.0)
        assert out.size == _img().size
