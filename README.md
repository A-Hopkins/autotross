
# AutoTross

**autotross** is a modular, C++-based flight software project designed for fixed-wing UAV drones, with a strong emphasis on safety, determinism, and extensibility. The project aims to support advanced longevity and autonomy through the integration of machine learning algorithms.

## Project Goals & Phases

1. **Deterministic Flight Software**  
   Develop a robust, safe, and deterministic flight control stack tailored for fixed-wing UAVs.
2. **Autonomous Long-Range Operations**  
   Implement support for waypoint navigation and long-range flight missions.
3. **Machine Learning Integration**  
   Experiment with and deploy ML-driven control algorithms to improve flight efficiency and longevity.


---
## Architecture Overview

- **Component-Based Software (CSCs):**  
  The system is organized into discrete Computer Software Components (CSCs) to modularize and clearly define all system responsibilities. Each CSC owns a specific behavior of the UAV and takes responsibility for the inputs it receives and the outputs it generates. There is also a specialized Safety Monitor CSC that provides independent safety verifications of each CSC that may override and demand known states. This will work in conjunction with the protocore library's built in state management and heart beat for tasking so that it may decide UAV behavior in the case of task watch dog failure.

## CSCs

AutoTross is designed around modular **Computer Software Components (CSCs)**, which represent the major functional units of the UAV system. These CSCs communicate with each other using an **event-driven messaging system** to ensure decoupled and scalable interactions.

Each CSC implements input and output sanity checks to ensure safety at the CSC level

### Mission Manager CSC

The Mission Manager CSC is responsible for controlling the overall progression of the autonomous mission. It maintains the mission state machine, which transitions through various flight phases such as takeoff, climb, cruise, waypoint transit, and landing. It ingests waypoint lists and mission sequences, which are uploaded via external interfaces, and tracks current progress through these waypoints by subscribing to navigational feedback from the Navigation CSC. It also receives external commands such as pause, resume, or abort, and acts upon high-level fault or degraded-mode flags issued by supervisory components like the Safety Monitor CSC. The Mission Manager evaluates whether the vehicle is in a nominal state to execute each phase, and when all preconditions are met, it issues navigation objectives and desired behaviors to downstream components. These behaviors may include transitioning to the next waypoint, loitering, returning to launch, or initiating landing. It also monitors the success or failure of executed directives and updates its internal state accordingly.

**Inputs**
  - Current navigation progress and position status
  - External commands (e.g., from ground control or onboard manual input)
  - Uploaded mission item list (waypoints, sequences, behavioral triggers)
  - Fault and degraded-state flags from supervisory components

**Outputs**
  - Navigation and behavioral directives corresponding to current mission phase
  - Mission execution state, active mode, and progress reporting

**Goal**
To govern the execution of mission phases by evaluating vehicle state, interpreting mission plans, and issuing phase-appropriate navigation and behavioral commands. Ensures orderly progression of autonomous behavior under normal and degraded conditions.

---

### Navigation CSC

The Navigation CSC is responsible for converting high-level navigation objectives into real-time guidance trajectories and control setpoints. It receives target objectives in the form of positional goals, behavioral directives, or path segments from the Mission Manager CSC. To achieve these objectives, it maintains an internal path-tracking module capable of computing time-consistent trajectories within known vehicle kinematic and dynamic constraints. The component continuously fuses the latest vehicle state estimates—position, velocity, attitude—from the State Estimation CSC to compute guidance vectors such as desired airspeed, heading, or waypoint intercept logic. These guidance outputs are converted into position, velocity, or attitude setpoints, depending on the current flight control mode, and are transmitted to the Flight Controller CSC for execution. The Navigation CSC also includes logic to assess whether the commanded objective has been achieved, either by proximity, time, or progression through a path segment, and reports this completion status back to the Mission Manager. Additionally, it performs feasibility checks to avoid generating unachievable trajectories, such as excessive climb rates or turns beyond aircraft bank limits. If such conditions are detected, it flags errors upstream for handling by mission-level logic or failsafe systems.

**Inputs**
  - High-level navigation objectives (e.g., position goals, path segments, behavioral modes)
  - Real-time vehicle state (position, velocity, attitude) from the State Estimation CSC

**Outputs**
  - Guidance setpoints in position, velocity, or attitude form, depending on control mode
  - Progress and status feedback indicating whether objectives have been reached or are infeasible

**Goal**
To convert high-level navigation goals into safe, feasible, and dynamically achievable guidance setpoints, while continuously tracking and reporting progress against these objectives.


---

### State Estimation CSC
The State Estimation CSC is responsible for producing a consistent, time-aligned estimate of the UAV's global position, velocity, orientation, and horizontal wind velocity by fusing multiple asynchronous and noisy sensor sources. It operates an Extended Kalman Filter (EKF) with a 12-dimensional state vector comprising 3D position, 3D velocity, 4D orientation (quaternion), and 2D horizontal wind velocity in the global frame. The estimator receives raw IMU data (angular velocity and linear acceleration), global position and velocity measurements (e.g., from GPS), differential pressure from the airspeed sensor, and optional barometric altitude. It corrects for measurement delays and asynchronous update rates through a prediction-correction cycle, using a discretized rigid-body motion model for propagation.

Wind velocity is estimated by comparing true airspeed, derived from differential pressure and estimated air density, with ground-relative velocity from GPS and orientation. The estimator applies data gating and sanity checks to reject low-confidence or physically invalid measurements. Its outputs are time-stamped, consistent state estimates published to all downstream consumers including the Flight Controller, Navigation, Safety Monitor, and Telemetry CSCs.
**Inputs:**
 - Angular velocity and linear acceleration (from IMU)
 - Global position and velocity (from GPS or equivalent)
 - Airspeed differential pressure and temperature
 - Altitude vertical velocity and position

**Outputs:**
 - Global position and velocity
 - Orientation as quaternion
 - Horizontal wind velocity estimate
 - Time-aligned full state estimate for all downstream CSCs

**Goal:**
To maintain an accurate and robust estimate of the UAV's inertial and global state for all flight phases using sensor fusion via EKF. The fused state supports closed-loop control, navigation logic, and safety supervision.

---

### Flight Controller CSC
The Flight Controller CSC is responsible for translating guidance setpoints and estimated state information into real-time actuator commands that control the UAV's motion. It executes a cascaded control architecture consisting of attitude control and angular rate control loops. The component consumes real-time orientation, velocity, and position from the Estimation CSC and directly receives high-rate angular velocity measurements from the IMU to support stable and low-latency inner-loop rate control.

The controller receives position, velocity, or attitude setpoints from the Navigation CSC. It computes desired angular rates and thrust vectors to track these setpoints, then runs a rate controller that compares desired vs. measured angular velocity to produce torque or control surface deflection commands. The output is a set of actuator commands, such as normalized surface deflection values or motor throttle/RPM settings, which are passed to the HAL CSC for hardware-specific conversion and execution.

The controller applies control effectiveness limits, actuator saturation handling, and constraint enforcement to ensure safe actuation under nominal and degraded conditions. Its control logic is tuned to remain robust under disturbances such as wind, model mismatch, and transient estimation noise.

**Inputs:**
 - Navigation setpoints (position, velocity, or attitude goals)
 - Estimated state (position, velocity, orientation, wind) from Estimation CSC
 - Angular velocity from IMU (high-rate input)

**Outputs:**
 - Actuator commands: control surface positions and motor thrust/RPM
 - Optional: controller status, actuator saturation flags

**Goal:**
To convert navigation objectives into stable and responsive actuator-level commands using real-time control loops. The controller ensures the UAV tracks desired trajectories while remaining within aerodynamic and physical limits.
---

### Safety Monitor CSC
The Safety Monitor CSC is an independent supervisory component responsible for monitoring the health, behavior, and state consistency of all critical UAV CSCs. It operates as a system-level fault detection and mitigation unit. Its primary function is to validate that all subsystems are operating within nominal bounds and to enforce global safety constraints, such as failsafe actions, override behaviors, and mission abort conditions. It does this by subscribing to all relevant CSC outputs—including state estimates, actuator commands, battery status, navigation progress, mission state, and heartbeat messages—to assess overall system integrity.

The Safety Monitor does not control the vehicle directly, but issues override commands when unsafe conditions are detected. These may include triggering an emergency loiter, return-to-launch (RTL), flight termination, or forced landing. It performs logical and temporal consistency checks across subsystems and uses watchdog timeouts to identify silent failures or liveness violations. The underlying fault model includes transient errors, systemic failures (e.g., estimator divergence, battery brownout), and inter-CSC disagreement.

The Safety Monitor operates on the following principles:

  - Input Validation: Range checks, cross-component agreement (e.g., comparing velocity from GPS vs. state estimate)
  - Temporal Monitoring: Ensures data freshness using Protocore heartbeat signals for all critical CSCs
  - Fault Classification: Differentiates between recoverable, degraded, and catastrophic failures
  - Override Execution: Issues a predefined safety directive (e.g., enter failsafe loiter) to the Flight Controller CSC

**Inputs:**
 - Estimated state from the Estimation CSC
 - Navigation status and progress
 - Mission state and active directives
 - Battery status (voltage, current, thermal)
 - IMU health (e.g., stale data, physical plausibility)
 - Heartbeat signals from all safety-critical CSCs

**Outputs:**
 - Triggers emergency states (e.g., emergency loiter, RTL)
 - Logs system-level safety violations

**Goal:**
To detect safety violations or subsystem faults in real time and to enforce system-wide overrides or fail-safe behaviors to maintain flight integrity and prevent loss of vehicle. Acts as the final authority on system health, independent of individual CSC logic.

---

### Battery Management CSC
The Battery Management CSC is responsible for monitoring the UAV's electrical power system, estimating available energy, and detecting unsafe power conditions that could impact mission success or system integrity. It continuously measures battery voltage, current draw, and temperature (if available) to compute state-of-charge (SoC), power consumption rate, and thermal load. Using these inputs, it performs energy forecasting to estimate time remaining and minimum required reserve capacity for safe mission termination.

The component maintains thresholds for undervoltage, overcurrent, and overtemperature conditions, and raises warnings or fault flags when limits are violated. These warnings are consumed by the Safety Monitor CSC and can influence mission execution logic in the Mission Manager CSC (e.g., triggering early return-to-launch). It can also distinguish between fast transients (e.g., short-term voltage drop under load) and sustained violations (e.g., battery exhaustion), allowing differentiated handling of recoverable vs. critical conditions.

In simulation, battery state may be derived from physics-based plugins or sensor emulation. On hardware, the Battery Management CSC would interface with the actual battery monitor IC or ESC telemetry data.
**Inputs:**
- Raw voltage reading
- Current draw measurement
- Thermal status

**Outputs:**
- State of charge, estimated time remaining
- Battery health status (normal, warning, critical)

**Goal:**
To provide reliable real-time monitoring and prediction of battery performance, enabling energy-aware flight behavior and system safety enforcement under power-constrained conditions.

---

### 📡 Telemetry CSC
The Telemetry CSC is responsible for aggregating, prioritizing, formatting, and transmitting system state and status data to an external operator or ground control station. It collects selected outputs from other CSCs—including state estimation, mission status, navigation progress, battery health, and fault diagnostics—and assembles them into structured telemetry packets for downlink.

The component supports configurable message rates, prioritization policies, and bandwidth constraints to ensure that critical information is delivered with low latency, while lower-priority data is sent opportunistically. In a degraded link scenario, it ensures that minimal safety and status telemetry is preserved by dropping nonessential packets.

**Inputs:**
- State estimation
- Mission status
- Navigation goal status
- Battery status
- System health

**Outputs:**
- formatted, prioritized outbound data for ground control

**Goal:**
To provide prioritized, timely, and operator-relevant telemetry of the UAV's state, status, and health during flight operations, supporting situational awareness, debugging, and safety monitoring.
---

## Getting Started

### Cloning the Repository

```bash
git clone --recurse-submodules https://github.com/A-Hopkins/autotross.git
```

If you've already cloned the repository without submodules, initialize and update the submodules with:

```bash
git submodule update --init --recursive
```

### Setting Up ProtoCore Submodule

autotross uses [ProtoCore](https://github.com/A-Hopkins/protocore) as a git submodule.  
To ensure ProtoCore is in an external folder (e.g., `../protocore`):

```bash
# Clone ProtoCore alongside autotross, in the parent directory
cd ..
git clone https://github.com/A-Hopkins/protocore.git
cd autotross
# Link the submodule to the external folder (optional advanced usage)
git submodule deinit protocore
rm -rf .git/modules/protocore
git config -f .gitmodules submodule.protocore.path ../protocore
git submodule update --init --recursive
```

### Building & Testing

- **Dependencies:**  
  - C++17 compiler  
  - [Google Test](https://github.com/google/googletest) (for unit tests)
  - [Gazebo](http://gazebosim.org/) (for simulation)


- **Building:**
```bash
mkdir build && cd build
cmake ..
make
```

- **Running Tests:**
  ```bash
  # Example placeholder, adjust as needed
  cd autotross
  mkdir build && cd build
  cmake ..
  make
  ctest
  ```

## Gazebo Simulation Setup

### Prerequisites

This project uses **Gazebo Harmonic (gz-sim 8)** and requires:

- A working installation of Gazebo Harmonic
- `gz` command-line tools (e.g., `gz sim`, `gz topic`, etc.)

Refer to [Gazebo Harmonic install instructions](https://gazebosim.org/docs/harmonic/install_ubuntu) for full setup.

### Running the Simulation

From the project root:

```bash
# Set resource path to find models
export GZ_SIM_RESOURCE_PATH=$PWD/gazebo/models

# Launch the simulation
gz sim gazebo/worlds/runway.sdf
```

### Model Attribution

This project uses an adapted version of the X-UAV Mini Talon model provided by the [ArduPilot SITL_Models project](https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/X-UAV_Mini_Talon.md).
All rights and credits for the original model and textures belong to the ArduPilot team. Please refer to their documentation and license for more details.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
