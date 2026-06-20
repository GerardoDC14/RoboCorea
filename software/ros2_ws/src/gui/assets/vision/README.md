# YOLO hazmat model

Place the trained hazmat detector here as a single file:

```
gui/assets/vision/
  best.onnx
  README.md  ← this file
```

The in-GUI **Hazmat** filter (OpenCV + ONNX Runtime) loads
`<gui share>/assets/vision/best.onnx` at startup, on a background thread, and
shares the one session across every video cell. Class names are read from the
model's embedded Ultralytics `names` metadata — no separate labels file.

A trained export ships in the legacy repo; copy it in with:

```bash
cp reference/TMR2026_Rescue/shared/vision/runs/hazmat_yolo11l/weights/best.onnx \
   software/ros2_ws/src/gui/assets/vision/best.onnx
```

(It is ~98 MB, so it is kept out of the source tree by default.) Re-run
`colcon build` afterwards so the install step copies it into the package share.

If the model is absent the GUI still runs — the Hazmat filter just overlays
"YOLO model not loaded" and every other filter/source works normally.

> ONNX Runtime is used (CUDA execution provider with CPU fallback), so the export
> opset is not constrained. The **library** (`libonnxruntime.so` +
> `onnxruntime_cxx_api.h`) is installed under `/usr/local` — see the legacy
> `shared/gui_ws/README.md` for the one-time install.
