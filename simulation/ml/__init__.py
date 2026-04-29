"""ML pipeline scaffolding — Python-only surface that CI exercises today.

Two parallel surfaces share the same registry and KPI conventions:

* **Detection**: `sar_targets`, `coco_annotator`, `image_augment`,
  `model_zoo`, `inference_logger`, `hard_example_miner`,
  `model_registry`, `kpi`. Optimises the perception model.
* **Control**: `waypoint_optimizer`, `waypoint_kpi`. Optimises the
  cascaded position/attitude PID gains for single-drone waypoint
  achievement.

The heavy components (PyTorch training, ONNX/TensorRT export, Jetson
deployment, Gazebo SAR world capture, RL trainers) live behind the
nightly lane (`docs/nightly_lane.md`).
"""
