"""Tests for the inference logger."""

import json

import pytest

from ml.inference_logger import (
    InferenceLogger, InferenceRecord, read_log,
)
from ml.model_zoo import Detection


class TestInferenceLogger:
    def _detection(self, conf=0.85):
        return Detection(category_id=1, category_name="person",
                         bbox=(0.0, 0.0, 10.0, 10.0), confidence=conf)

    def test_log_writes_jsonl(self, tmp_path):
        path = tmp_path / "infer.jsonl"
        with InferenceLogger(path, model_version="m1") as log:
            log.log([self._detection()], image_path="frame_0.png")
            log.log([self._detection(0.7), self._detection(0.4)],
                    image_path="frame_1.png",
                    metadata={"expected_targets": 2})
        lines = path.read_text(encoding="utf-8").strip().splitlines()
        assert len(lines) == 2
        rec0 = json.loads(lines[0])
        assert rec0["frame_id"] == 0
        assert rec0["model_version"] == "m1"
        assert rec0["image_path"] == "frame_0.png"
        assert len(rec0["detections"]) == 1

    def test_frame_ids_continue_across_reopens(self, tmp_path):
        path = tmp_path / "infer.jsonl"
        with InferenceLogger(path, model_version="m1") as log:
            log.log([self._detection()])
            log.log([self._detection()])
        with InferenceLogger(path, model_version="m1") as log:
            log.log([self._detection()])
        records = list(read_log(path))
        assert [r.frame_id for r in records] == [0, 1, 2]

    def test_read_log_recovers_records(self, tmp_path):
        path = tmp_path / "infer.jsonl"
        with InferenceLogger(path, model_version="m1") as log:
            log.log([self._detection(0.91)],
                    metadata={"weather": "rain"})
        recs = list(read_log(path))
        assert len(recs) == 1
        assert recs[0].metadata["weather"] == "rain"
        assert recs[0].detections[0].confidence == 0.91

    def test_read_log_skips_blank_lines(self, tmp_path):
        path = tmp_path / "infer.jsonl"
        path.write_text(
            json.dumps(InferenceRecord(
                frame_id=0, t_wall_s=0.0, model_version="m",
                detections=[]).to_dict()) + "\n\n",
            encoding="utf-8",
        )
        recs = list(read_log(path))
        assert len(recs) == 1

    def test_replay_materialises_records(self, tmp_path):
        from ml.inference_logger import replay
        path = tmp_path / "infer.jsonl"
        with InferenceLogger(path, model_version="m1") as log:
            log.log([self._detection(0.9)], image_path="a.png")
            log.log([self._detection(0.5)], image_path="b.png")
        out = replay(read_log(path))
        assert isinstance(out, list)
        assert [r.image_path for r in out] == ["a.png", "b.png"]

    def test_record_round_trip_preserves_detections(self, tmp_path):
        path = tmp_path / "infer.jsonl"
        with InferenceLogger(path, model_version="m1") as log:
            log.log(
                [self._detection(0.91), self._detection(0.42)],
                image_path="x.png",
                metadata={"weather": "rain"},
                t_wall_s=12345.0,
            )
        recs = list(read_log(path))
        assert len(recs) == 1
        assert recs[0].t_wall_s == 12345.0
        assert recs[0].metadata == {"weather": "rain"}
        confs = [d.confidence for d in recs[0].detections]
        assert confs == pytest.approx([0.91, 0.42])

    def test_from_dict_with_missing_optional_fields(self):
        rec = InferenceRecord.from_dict({
            "frame_id": 5,
            "t_wall_s": 1.0,
        })
        assert rec.frame_id == 5
        assert rec.model_version == "unknown"
        assert rec.image_path is None
        assert rec.metadata == {}
        assert rec.detections == []

    def test_tail_skips_corrupted_jsonl_line(self, tmp_path):
        # If a previous run died mid-write, _tail_max_frame_id should
        # ignore the broken line and resume frame numbering correctly.
        path = tmp_path / "infer.jsonl"
        good = json.dumps(InferenceRecord(
            frame_id=7, t_wall_s=0.0, model_version="m",
            detections=[]).to_dict())
        path.write_text(good + "\n{not json}\n", encoding="utf-8")
        with InferenceLogger(path, model_version="m") as log:
            rec = log.log([])
        assert rec.frame_id == 8

    def test_unfilled_metadata_serialises_as_empty_dict(self, tmp_path):
        path = tmp_path / "infer.jsonl"
        with InferenceLogger(path, model_version="m") as log:
            log.log([self._detection()])
        recs = list(read_log(path))
        assert recs[0].metadata == {}
