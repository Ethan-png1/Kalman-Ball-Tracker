# Kalman Ball Tracker

A C++ demo of a Kalman filter tracking a ball as it bounces around a window in random motion. The filter only ever sees noisy position measurements — and loses them entirely on random frames — yet maintains a smooth estimate of where the ball is and where it's headed.

## What you see

The simulation window shows four things at ~60 fps:

| Marker | Meaning |
|---|---|
| White ball | Ground truth — bounces off walls with random acceleration jitter |
| Green cross | The measurement: true position + Gaussian noise (σ = 10 px) |
| Red circle | The Kalman filter's position estimate |
| Yellow arrow + circle | Prediction of where the ball will be 250 ms from now |

15% of frames the measurement randomly drops out ("MEASUREMENT LOST"). During dropouts the filter coasts on its motion model alone — watch the red circle keep tracking through the gap.

Press **ESC** to quit.

## How it works

The filter ([include/KalmanFilter.hpp](include/KalmanFilter.hpp), [src/kalmanFilter.cpp](src/kalmanFilter.cpp)) is a standard linear Kalman filter with a **constant-velocity model**:

- **State** `x = [px, py, vx, vy]ᵀ` — position and velocity in pixels.
- **Predict**: positions advance by `v·dt`; uncertainty `P` grows by the process noise `Q`.
- **Update**: the measurement `z = [px, py]` pulls the state toward what was observed, weighted by the Kalman gain (the balance between model trust and measurement trust).

`Q` uses the white-noise-acceleration model:

```
Q = σ_a² · | dt⁴/4    0    dt³/2    0   |
           |   0    dt⁴/4    0    dt³/2 |
           | dt³/2    0     dt²     0   |
           |   0    dt³/2    0     dt²  |
```

where `σ_a` is the standard deviation of the unmodeled acceleration. Wall bounces and the random jitter both violate constant velocity, so `σ_a` is set a bit above the simulation's true jitter to let the filter recover quickly after each bounce.

The yellow prediction is just the constant-velocity projection `p + v·Δt` with `Δt = 0.25 s`.

## Building

Requirements:

- CMake ≥ 3.14
- A C++17 compiler (tested with MSVC)
- OpenCV (must be findable by `find_package(OpenCV)`)
- Eigen is fetched automatically via CMake `FetchContent`

```sh
cmake -S . -B build
cmake --build build --config Release
./build/Release/kalman_tracker     # Windows: build\Release\kalman_tracker.exe
```

## Tuning

All the knobs live at the top of [src/main.cpp](src/main.cpp):

| Constant | Default | Effect |
|---|---|---|
| `MEAS_STD` | 10.0 | Measurement noise σ (px). Higher = noisier green crosses. |
| `ACCEL_STD` | 250.0 | True random acceleration of the ball (px/s²). Higher = more erratic motion. |
| `DROPOUT_PROB` | 0.15 | Per-frame chance the measurement is lost. |
| `PROCESS_ACCEL_STD` | 400.0 | Filter's assumed acceleration σ (px/s²). The key tradeoff knob. |
| `LOOKAHEAD_S` | 0.25 | How far into the future the yellow prediction projects. |

The interesting one is `PROCESS_ACCEL_STD`:

- **Too low** (e.g. 20): the filter over-trusts its model — the estimate is silky smooth but lags badly behind the ball, especially after bounces.
- **Too high** (e.g. 5000): the filter over-trusts measurements — the estimate snaps to every noisy green cross and jitters.
- **About right** (a bit above the ball's true `ACCEL_STD`): tight tracking with most of the measurement noise filtered out.

Note that the `KalmanFilter` constructor takes *variances*, not standard deviations — `main.cpp` squares these constants before passing them in.

## Project layout

```
include/KalmanFilter.hpp   # Filter interface (Eigen-based)
src/kalmanFilter.cpp       # Predict/update implementation
src/main.cpp               # Simulation, measurement model, visualization
CMakeLists.txt
```

## Demo
<img width="864" height="864" alt="download" src="https://github.com/user-attachments/assets/dbe68dd6-8f8f-4ba6-b758-bc5b89168791" />
