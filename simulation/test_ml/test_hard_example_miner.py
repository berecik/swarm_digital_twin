"""Tests for the hard-example miner."""

import pytest

from ml.hard_example_miner import (
    HardExample, MinerConfig, find_missed, find_uncertain, mine,
)
from ml.inference_logger import InferenceRecord
from ml.model_zoo import Detection


def _record(frame_id, detections, expected_targets=0,
            image_path="img.png"):
    return InferenceRecord(
        frame_id=frame_id, t_wall_s=0.0, model_version="m1",
        image_path=image_path,
        metadata={"expected_targets": expected_targets},
        detections=detections,
    )


def _det(conf):
    return Detection(category_id=1, category_name="person",
                     bbox=(0.0, 0.0, 10.0, 10.0), confidence=conf)


class TestUncertain:
    def test_finds_records_in_uncertainty_band(self):
        recs = [
            _record(1, [_det(0.45)]),    # in band
            _record(2, [_det(0.95)]),    # confident — not flagged
            _record(3, [_det(0.10)]),    # very low — not flagged
        ]
        out = find_uncertain(recs)
        assert [e.frame_id for e in out] == [1]
        assert all(isinstance(e, HardExample) for e in out)

    def test_score_higher_at_band_centre(self):
        # Default band [0.30, 0.60] → mid = 0.45.
        recs = [
            _record(1, [_det(0.45)]),    # exactly at mid
            _record(2, [_det(0.55)]),    # off-centre
        ]
        out = {e.frame_id: e.score for e in find_uncertain(recs)}
        assert out[1] > out[2]
        assert out[1] == pytest.approx(1.0)


class TestMissed:
    def test_zero_detections_with_expected_targets_flagged(self):
        recs = [
            _record(1, [], expected_targets=2),
            _record(2, [], expected_targets=0),    # nothing expected
            _record(3, [_det(0.9)], expected_targets=1),  # got it
        ]
        out = find_missed(recs)
        assert [e.frame_id for e in out] == [1]
        assert out[0].score == 2.0


class TestMine:
    def test_combines_and_sorts_descending(self):
        recs = [
            _record(1, [], expected_targets=3),     # missed: score 3.0
            _record(2, [_det(0.45)]),               # uncertain: score 1.0
            _record(3, [], expected_targets=2),     # missed: score 2.0
        ]
        out = mine(recs)
        # Strict descending order: 3.0 > 2.0 > 1.0.
        assert [e.frame_id for e in out] == [1, 3, 2]
        assert out[0].score >= out[1].score >= out[2].score

    def test_custom_config_widens_uncertainty(self):
        cfg = MinerConfig(low_conf=0.1, high_conf=0.9)
        recs = [_record(1, [_det(0.85)])]
        out = find_uncertain(recs, cfg)
        assert len(out) == 1


class TestEdgeCases:
    def test_mine_empty_records_returns_empty(self):
        assert mine([]) == []

    def test_find_missed_ignores_records_with_detections(self):
        # Even if expected_targets is set, a frame with detections is
        # not "missed" — the miner only flags zero-detection frames.
        recs = [_record(1, [_det(0.95)], expected_targets=3)]
        assert find_missed(recs) == []

    def test_find_missed_default_zero_when_metadata_missing(self):
        rec = InferenceRecord(
            frame_id=1, t_wall_s=0.0, model_version="m",
            metadata={},  # no expected_targets key
            detections=[],
        )
        assert find_missed([rec]) == []

    def test_hard_example_to_dict(self):
        ex = HardExample(frame_id=42, image_path="foo.png",
                         reason="uncertain", score=0.83)
        out = ex.to_dict()
        assert out == {
            "frame_id": 42,
            "image_path": "foo.png",
            "reason": "uncertain",
            "score": 0.83,
        }

    def test_uncertain_score_capped_at_one(self):
        # A detection at the exact band centre scores 1.0; the formula
        # should never exceed 1.0 even for that input.
        recs = [_record(1, [_det(0.45)])]  # exactly at midpoint
        out = find_uncertain(recs)
        assert out[0].score == pytest.approx(1.0)
        assert out[0].score <= 1.0

    def test_uncertain_picks_worst_when_multiple_in_band(self):
        # When a frame has multiple uncertain detections, the score
        # should reflect the most-uncertain one (closest to band centre).
        recs = [_record(1, [_det(0.55), _det(0.45)])]
        out = find_uncertain(recs)
        assert len(out) == 1
        # 0.45 is at midpoint → score = 1.0 (would beat 0.55 → 0.66).
        assert out[0].score == pytest.approx(1.0)
