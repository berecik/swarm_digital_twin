"""Tests for the model registry."""

import pytest

from ml.model_registry import ModelEntry, ModelRegistry


class TestModelRegistry:
    def _entry(self, version="v1", parent=None, kpis=None):
        return ModelEntry(
            version=version, backend="yolo",
            weights_path=f"models/{version}.pt",
            trained_at_s=1700000000.0,
            parent_version=parent,
            kpis=kpis or {"mAP_50": 0.78, "recall": 0.86, "inference_ms": 42.0},
            notes="",
        )

    def test_register_persists_to_disk(self, tmp_path):
        path = tmp_path / "registry.json"
        reg = ModelRegistry(path)
        reg.register(self._entry("v1"))
        # Reload — entry must be present.
        reg2 = ModelRegistry(path)
        assert [e.version for e in reg2.all()] == ["v1"]

    def test_duplicate_version_raises(self, tmp_path):
        reg = ModelRegistry(tmp_path / "r.json")
        reg.register(self._entry("v1"))
        with pytest.raises(ValueError, match="already registered"):
            reg.register(self._entry("v1"))

    def test_get_unknown_raises(self, tmp_path):
        reg = ModelRegistry(tmp_path / "r.json")
        with pytest.raises(KeyError, match="no model with version"):
            reg.get("ghost")

    def test_latest_returns_last_registered(self, tmp_path):
        reg = ModelRegistry(tmp_path / "r.json")
        assert reg.latest() is None
        reg.register(self._entry("v1"))
        reg.register(self._entry("v2"))
        assert reg.latest().version == "v2"

    def test_best_by_kpi(self, tmp_path):
        reg = ModelRegistry(tmp_path / "r.json")
        reg.register(self._entry("v1", kpis={"mAP_50": 0.70}))
        reg.register(self._entry("v2", kpis={"mAP_50": 0.85}))
        reg.register(self._entry("v3", kpis={"mAP_50": 0.80}))
        assert reg.best_by_kpi("mAP_50").version == "v2"
        assert reg.best_by_kpi("does_not_exist") is None

    def test_lineage_walks_parent_pointers(self, tmp_path):
        reg = ModelRegistry(tmp_path / "r.json")
        reg.register(self._entry("v1"))
        reg.register(self._entry("v2", parent="v1"))
        reg.register(self._entry("v3", parent="v2"))
        chain = [e.version for e in reg.lineage("v3")]
        assert chain == ["v3", "v2", "v1"]

    def test_lineage_detects_hand_edited_cycle(self, tmp_path):
        """If the JSON file is hand-edited to introduce a cycle, lineage
        must raise rather than spin forever."""
        import json
        path = tmp_path / "r.json"
        path.write_text(json.dumps({
            "models": [
                {"version": "a", "backend": "yolo", "weights_path": "a.pt",
                 "trained_at_s": 0.0, "parent_version": "b", "kpis": {},
                 "notes": ""},
                {"version": "b", "backend": "yolo", "weights_path": "b.pt",
                 "trained_at_s": 0.0, "parent_version": "a", "kpis": {},
                 "notes": ""},
            ]
        }), encoding="utf-8")
        reg = ModelRegistry(path)
        with pytest.raises(ValueError, match="cycle detected"):
            reg.lineage("a")

    def test_lineage_unknown_version_raises_keyerror(self, tmp_path):
        reg = ModelRegistry(tmp_path / "r.json")
        with pytest.raises(KeyError, match="no model with version"):
            reg.lineage("ghost")

    def test_lineage_single_root_entry(self, tmp_path):
        reg = ModelRegistry(tmp_path / "r.json")
        reg.register(self._entry("v1"))
        chain = reg.lineage("v1")
        assert [e.version for e in chain] == ["v1"]
        assert chain[0].parent_version is None

    def test_all_returns_independent_copy(self, tmp_path):
        reg = ModelRegistry(tmp_path / "r.json")
        reg.register(self._entry("v1"))
        snap = reg.all()
        snap.clear()
        # Mutating the snapshot must not affect the registry.
        assert len(reg.all()) == 1

    def test_model_entry_to_dict_round_trip(self):
        e = self._entry("vX", parent="vW", kpis={"mAP_50": 0.7})
        d = e.to_dict()
        assert d["version"] == "vX"
        assert d["parent_version"] == "vW"
        assert d["kpis"] == {"mAP_50": 0.7}
        # Reconstructing from the dict yields the same entry.
        e2 = ModelEntry(**d)
        assert e2 == e

    def test_load_empty_file_yields_empty_registry(self, tmp_path):
        path = tmp_path / "empty.json"
        path.write_text("", encoding="utf-8")
        reg = ModelRegistry(path)
        assert reg.all() == []
        assert reg.latest() is None

    def test_register_with_unknown_parent_is_allowed(self, tmp_path):
        # Registry doesn't enforce parent existence at write-time —
        # lineage walking will surface the broken pointer instead.
        reg = ModelRegistry(tmp_path / "r.json")
        reg.register(self._entry("v1", parent="missing_root"))
        with pytest.raises(KeyError):
            reg.lineage("v1")
