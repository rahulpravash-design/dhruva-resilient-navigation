# Technology stack (what is used today)

| Layer | Technology | Status |
|---|---|---|
| Engine | Python 3.12, NumPy, pandas | IMPLEMENTED |
| State estimation | own error-state EKF (8 states), NHC, ZUPT/ZARU | IMPLEMENTED |
| Speed model | Keras 1D-CNN (47,394 parameters) exported to float16 TFLite (`models/speednet.tflite`, about 108 KB), run through `tf.lite.Interpreter` | IMPLEMENTED, trained on SYNTHETIC data |
| Integrity | own monitor: chi-square NIS tests, CUSUM, re-entry validation | IMPLEMENTED |
| Map matching / routing | own graph, HMM/Viterbi matcher and heap-based A* router (no graph library) | NOT WIRED into the engine |
| Evaluation | pandas/NumPy, joblib for parallel runs, PyYAML config (`configs/eval.yaml`) | IMPLEMENTED, full run not committed |
| Tests / lint | pytest, pytest-xdist, ruff, jsonschema | IMPLEMENTED |
| Web demo | one static `web/index.html` (plain JavaScript, canvas drawing), data in `web/data/demo.js`, served by `python -m http.server`. No framework, no build step, no CDN, no map tiles. | IMPLEMENTED |
| Android | Kotlin, Jetpack Compose, Android sensor and location APIs, MapLibre | PLANNED (SDK not installed, no code) |
| Kotlin engine | pure Kotlin/JVM port of the Python engine, with a parity test | PLANNED |
| Web app (React, Vite, TypeScript) | replacement for the static page | PLANNED, only if time allows; the static page is the working deliverable |

Dependencies are listed in `requirements.txt` and `requirements-ml.txt`. The web demo needs only Python 3 to serve it.
No network access is needed at runtime.
