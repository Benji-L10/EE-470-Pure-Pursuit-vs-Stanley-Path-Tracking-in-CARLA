# Pure Pursuit vs. Stanley Path Tracking in CARLA

## Project Overview

This project compares two classical lateral path-tracking controllers, **Pure Pursuit** and **Stanley control**, in the CARLA autonomous-driving simulator. The goal is to evaluate how each controller performs under different vehicle speeds and path geometries.

Accurate path tracking is a core requirement for autonomous vehicles because the vehicle must follow a desired trajectory while minimizing cross-track error, heading error, and unstable steering behavior. This project focuses on the lateral-control portion of the autonomous-driving stack.

The project was completed for **EE 470: Planning and Control for Autonomous Vehicles** at California Polytechnic State University, San Luis Obispo.

## Team Members

* Alex Nguyen
* Benjamin Ly

## Project Objectives

The objectives of this project are to:

* implement Pure Pursuit and Stanley path-tracking controllers;
* integrate both controllers with the CARLA Python API;
* generate repeatable reference routes from the CARLA road network;
* test both controllers under identical simulation conditions;
* evaluate performance on two qualitatively different path geometries;
* compare controller performance at 5, 10, and 15 m/s; and
* analyze tracking accuracy and steering smoothness.

The comparison focuses on:

* mean absolute cross-track error;
* RMS cross-track error;
* maximum cross-track error;
* heading error;
* steering rate;
* speed tracking; and
* controller behavior through gradual curves and sharp turns.

## Simulation Environment

The experiments were performed using:

* CARLA Simulator
* CARLA Python API
* Python 3
* NumPy
* pandas
* Matplotlib
* PyYAML
* CARLA Global Route Planner

CARLA was operated in synchronous mode with a fixed simulation time step. Both controllers used the same:

* vehicle model;
* vehicle spawn conditions;
* reference path;
* target speed;
* simulation time step;
* longitudinal PI speed controller;
* steering limits; and
* termination criteria.

This ensured that the lateral controller was the primary difference between paired experiments.

## Controllers

### Pure Pursuit Controller

Pure Pursuit is a geometric path-tracking controller that selects a look-ahead point on the reference path and computes the steering angle required to drive toward that point.

The steering command is

```math
\delta =
\tan^{-1}
\left(
\frac{2L\sin(\alpha)}{L_d}
\right)
```

where:

* (L) is the vehicle wheelbase;
* (\alpha) is the angle between the vehicle heading and the look-ahead point; and
* (L_d) is the look-ahead distance.

The look-ahead distance increases with vehicle speed:

```math
L_d = \max(L_{d,\min}, K_v v)
```

The implementation stores the current path progress and searches forward from the most recent closest waypoint. This prevents the controller from selecting previously passed points.

### Stanley Controller

Stanley control combines heading error and signed cross-track error at the front axle.

The steering command is

```math
\delta =
\psi_e +
\tan^{-1}
\left(
\frac{k e_c}{v+k_s}
\right)
```

where:

* (\psi_e) is the heading error;
* (e_c) is the signed cross-track error;
* (k) is the Stanley gain;
* (k_s) is a low-speed softening constant; and
* (v) is the vehicle speed.

Stanley directly corrects vehicle alignment and lateral displacement from the path.

## Experiment Design

Two reference-path geometries were used.

### Smooth Curvy Path

The smooth-curvy route contains:

* long straight segments;
* gradual changes in curvature; and
* rounded transitions between road sections.

### Sharp-Turn Path

The sharp-turn route contains:

* multiple approximately right-angle turns;
* rapid changes in reference heading; and
* shorter recovery distances between turns.

Each controller was tested at:

* 5 m/s;
* 10 m/s; and
* 15 m/s.

The complete experiment matrix contained:

```text
2 controllers × 2 paths × 3 speeds = 12 runs
```

## Repository Structure

```text
EE-470-Pure-Pursuit-vs-Stanley-Path-Tracking-in-CARLA/
├── config/
│   └── experiment_config.yaml
├── controllers/
│   ├── __init__.py
│   ├── pure_pursuit.py
│   └── stanley.py
├── data/
│   ├── smooth_curvy/
│   ├── sharp_turns/
│   └── batch_report.json
├── experiments/
│   ├── __init__.py
│   ├── generate_paths.py
│   ├── run_experiment.py
│   └── run_all_experiments.py
├── paths/
│   ├── smooth_curvy.csv
│   └── sharp_turns.csv
├── plots/
│   ├── report/
│   ├── smooth_curvy_5ms/
│   └── sharp_turns_5ms/
├── utils/
│   ├── __init__.py
│   ├── logger.py
│   ├── metrics.py
│   └── plotting.py
├── .gitignore
├── generate_path.py
└── README.md
```

## Installation

### 1. Clone the Repository

```powershell
git clone <repository-url>
cd EE-470-Pure-Pursuit-vs-Stanley-Path-Tracking-in-CARLA
```

### 2. Create a Virtual Environment

On Windows PowerShell:

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
```

### 3. Install Python Dependencies

```powershell
python -m pip install numpy pandas matplotlib pyyaml
```

The CARLA Python package must also be available in the same Python environment.

Test the installation with:

```powershell
python -c "import carla; print('CARLA Python API imported successfully')"
```

If this command fails, install or add the CARLA Python API package that matches the installed CARLA simulator version.

## Running CARLA

Start the CARLA simulator before running any experiment.

The Python scripts connect to the CARLA server using:

```text
Host: 127.0.0.1
Port: 2000
```

Wait until the CARLA map is fully loaded before starting an experiment.

## Generating Reference Paths

The reference paths are generated using CARLA's Global Route Planner.

From the repository root, run:

```powershell
python experiments/generate_paths.py --map Town04
```

This creates:

```text
paths/smooth_curvy.csv
paths/sharp_turns.csv
```

The generated files contain CARLA world coordinates and route metadata.

A larger route search can be performed with:

```powershell
python experiments/generate_paths.py --map Town04 --samples 500 --min-length 120 --max-length 500
```

The generated routes should be visually inspected in CARLA before collecting final experiment data.

## Running One Experiment

The `run_experiment.py` script runs one controller, path, and speed combination.

### Pure Pursuit Example

```powershell
python experiments/run_experiment.py `
    --controller pure_pursuit `
    --path-csv paths/smooth_curvy.csv `
    --path-name smooth_curvy `
    --speed 5 `
    --map Town04 `
    --spectator-follow
```

### Stanley Example

```powershell
python experiments/run_experiment.py `
    --controller stanley `
    --path-csv paths/sharp_turns.csv `
    --path-name sharp_turns `
    --speed 10 `
    --map Town04 `
    --spectator-follow
```

PowerShell uses the backtick character for line continuation. The same command can also be entered on one line.

The experiment runner:

1. connects to CARLA;
2. enables synchronous mode;
3. loads the selected map;
4. loads the reference path;
5. spawns the ego vehicle;
6. runs the selected lateral controller;
7. maintains the requested speed using a PI controller;
8. logs vehicle and controller data;
9. detects collisions and excessive path error; and
10. saves the experiment results.

## Running the Complete Automated Test

The `run_all_experiments.py` script runs the complete 12-run experiment matrix.

Before starting all experiments, verify the commands with:

```powershell
python experiments/run_all_experiments.py --dry-run
```

Run the complete experiment set with:

```powershell
python experiments/run_all_experiments.py `
    --map Town04 `
    --spectator-follow `
    --continue-on-error
```

To test only the 5 m/s cases:

```powershell
python experiments/run_all_experiments.py `
    --map Town04 `
    --speeds 5 `
    --spectator-follow
```

To test one path at all speeds:

```powershell
python experiments/run_all_experiments.py `
    --map Town04 `
    --paths smooth_curvy `
    --speeds 5 10 15 `
    --spectator-follow
```

The batch script runs each experiment as a separate process. A new vehicle is spawned for each run, and the results are stored in the corresponding data directory.

## Experiment Outputs

Each run creates a CSV file and a JSON summary.

Example:

```text
data/smooth_curvy/
├── pure_pursuit_5ms.csv
├── pure_pursuit_5ms.summary.json
├── stanley_5ms.csv
└── stanley_5ms.summary.json
```

The CSV files contain:

* simulation time;
* vehicle position;
* vehicle yaw;
* vehicle speed;
* target speed;
* steering angle;
* normalized steering input;
* throttle;
* brake;
* closest reference point;
* closest path index;
* cross-track error;
* heading error;
* distance to the goal; and
* collision status.

The batch runner also creates:

```text
data/batch_report.json
```

This report records whether each automated experiment completed successfully.

## Generating Plots

The plotting script creates report-ready comparison figures using all experiment CSV files.

Run:

```powershell
python utils/plotting.py `
    --data-dir data `
    --speed 10 `
    --output-dir plots/report
```

The selected speed is used for the trajectory and cross-track-error figures. All speeds are included in the summary metrics.

The script generates:

```text
plots/report/
├── trajectory_comparison_10ms.png
├── cross_track_error_10ms.png
├── metric_summary.png
└── all_summary_metrics.csv
```

To create the trajectory and cross-track-error figures using 5 m/s:

```powershell
python utils/plotting.py --data-dir data --speed 5 --output-dir plots/report
```

To use 15 m/s:

```powershell
python utils/plotting.py --data-dir data --speed 15 --output-dir plots/report
```

## Evaluation Metrics

### RMS Cross-Track Error

Tracking accuracy is evaluated using:

```math
e_{\mathrm{RMS}}
=
\sqrt{
\frac{1}{N}
\sum_{i=1}^{N} e_{c,i}^{2}
}
```

A lower RMS cross-track error indicates more accurate path tracking.

### Mean Steering Rate

Steering activity is evaluated using:

```math
\overline{\dot{\delta}}
=
\frac{1}{N-1}
\sum_{i=2}^{N}
\left|
\frac{\delta_i-\delta_{i-1}}{\Delta t}
\right|
```

A lower mean steering rate indicates smoother steering behavior.

## Main Results

The experimental results showed that:

* both controllers successfully followed both CARLA routes;
* Pure Pursuit achieved lower RMS cross-track error on the smooth-curvy path at all tested speeds;
* Pure Pursuit performed substantially better on the sharp-turn path at 5 m/s;
* Stanley achieved slightly lower RMS error on the sharp-turn path at 10 and 15 m/s;
* Pure Pursuit produced lower steering activity in every experiment; and
* tracking error and steering activity increased as vehicle speed increased.

Overall, Pure Pursuit provided the best balance between tracking accuracy and steering smoothness for the selected vehicle, paths, and controller parameters.

## Common Issues

### CARLA Import Error

```text
ModuleNotFoundError: No module named 'carla'
```

Confirm that the CARLA Python API is installed in the active virtual environment.

### Global Route Planner Import Error

```text
Could not import CARLA or GlobalRoutePlanner
```

Add the CARLA Python API and `agents` directory to `PYTHONPATH`, or install the correct CARLA Python package.

### CARLA Connection Timeout

```text
time-out while waiting for the simulator
```

Confirm that:

* CARLA is running;
* the map has finished loading;
* the server is using port 2000; and
* the host address is correct.

### Vehicle Immediately Leaves the Road

Confirm that:

* the path was generated from the loaded CARLA map;
* the selected map matches the path CSV;
* the vehicle spawned near the beginning of the path; and
* the steering direction is consistent with CARLA's steering convention.

### Missing Plotting Packages

Install the required packages with:

```powershell
python -m pip install numpy pandas matplotlib
```

## Future Work

Possible extensions include:

* speed-dependent gain scheduling;
* adaptive Pure Pursuit look-ahead distance;
* automatic Stanley gain tuning;
* localization and sensor noise;
* steering actuator delay;
* changes in road friction;
* surrounding traffic and dynamic obstacles;
* repeated statistical trials; and
* comparison with model predictive control.

## Course Deliverables

The final project deliverables include:

* this GitHub repository;
* Python source code;
* generated experiment data;
* controller comparison plots;
* a 3-minute video presentation; and
* a four-page IEEE conference-format report.