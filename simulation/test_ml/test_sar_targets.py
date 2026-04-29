"""Tests for the SAR target catalogue."""

import pytest

from ml.sar_targets import (
    SAR_TARGETS, SARTarget, coco_categories, find, total_count,
)


class TestSARTargets:
    def test_at_least_20_target_classes(self):
        # Acceptance: 20+ target models in the SAR world.
        assert total_count() >= 20

    def test_categories_cover_required_supercategories(self):
        sup = {tgt.supercategory for tgt in SAR_TARGETS.values()}
        assert {"person", "vehicle", "equipment", "hazard"}.issubset(sup)

    def test_coco_category_ids_unique(self):
        ids = [t.category_id for t in SAR_TARGETS.values()]
        assert len(ids) == len(set(ids))

    def test_find_returns_target(self):
        person = find("person")
        assert isinstance(person, SARTarget)
        assert person.name == "person"
        assert person.category_id == 1

    def test_find_unknown_raises(self):
        with pytest.raises(KeyError, match="unknown SAR target"):
            find("does_not_exist")

    def test_coco_categories_block_shape(self):
        cats = coco_categories()
        assert all({"id", "name", "supercategory"} <= c.keys() for c in cats)
        # Sorted by id.
        ids = [c["id"] for c in cats]
        assert ids == sorted(ids)

    def test_footprint_is_three_tuple(self):
        for t in SAR_TARGETS.values():
            assert len(t.footprint_m) == 3
            assert all(v > 0 for v in t.footprint_m), t.name

    def test_target_is_frozen(self):
        person = find("person")
        with pytest.raises(Exception):
            person.name = "robot"  # type: ignore[misc]

    def test_canonical_yaw_defaults_to_zero(self):
        # The catalogue does not override canonical_yaw, so every entry
        # should default to 0.0 — keeps the projector deterministic.
        for t in SAR_TARGETS.values():
            assert t.canonical_yaw == 0.0, t.name

    def test_coco_categories_match_sar_targets_count(self):
        assert len(coco_categories()) == total_count()
