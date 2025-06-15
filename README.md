
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

### 🚀 Mission Manager CSC

**Inputs:**
- Current navigation progress and status
- Externally issued commands (GCS, manual input)
- Sequence of mission items (waypoints, actions)
- High-level fault and degraded mode info

**Outputs:**
- `MissionDirectiveMsg` — next waypoint or behavior to execute
- `MissionStatusMsg` — current mode, directive, and state for telemetry/logging

**Goal:**
To coordinate and execute the mission lifecycle by interpreting navigation status, mission plans, and health reports. It owns the high-level behavior and flight phase logic, and directs other CSCs accordingly.

---

### 🧭 Navigation CSC

**Inputs:**
- High-level goal to navigate to
- Current position, velocity, attitude

**Outputs:**
- Velocity, attitude, or position setpoints for Flight Controller
- `NavStatusMsg` — current goal progress, ETA, and health info

**Goal:**
To plan and track flight paths that satisfy mission directives. Converts global goals into feasible, dynamic setpoints, ensuring flight path feasibility and goal completion feedback.

---

### 🧠 State Estimation CSC

**Inputs:**
- High-rate inertial data (acceleration, angular velocity)
- Global position and velocity
- Sltitude from pressure
- Heading reference
- Indicated airspeed

**Outputs:**
- `StateEstimateMsg` — fused position, velocity, orientation, wind, and bias estimates

**Goal:**
To maintain a real-time, drift-corrected estimate of the UAV's full 6-DOF state by fusing multiple sensor sources. Provides the foundational truth required for guidance and control.

---

### 🎯 Flight Controller CSC

**Inputs:**
- Position, velocity, or attitude goals
- `StateEstimateMsg` — fused vehicle state

**Outputs:**
- `ActuatorCommandMsg` — throttle and control surface commands sent to the HAL

**Goal:**
To convert guidance setpoints into low-level actuator commands using cascaded control loops (rate, attitude, velocity). Ensures stable and responsive control within flight envelope constraints.

---

### 🛑 Safety Monitor CSC

**Inputs:**
- All the CSC inputs to monitor

**Outputs:**
-Triggers emergency states (e.g., emergency loiter, RTL)
- Logs system-level safety violations

**Goal:**
To supervise system-wide safety conditions, issue global overrides, and act as an independent failsafe authority. Serves as the last line of defense in fault conditions.

---

### 🔋 Battery Management CSC

**Inputs:**
- Raw voltage reading
- Current draw measurement
- Thermal status

**Outputs:**
- State of charge, estimated time remaining
- `BatteryHealthMsg` — voltage drop, overcurrent, or overtemp warnings

**Goal:**
To track and predict battery energy state and power consumption, enabling safe energy-aware behavior. Ensures mission aborts and landings occur before power exhaustion.

---

### 📡 Telemetry CSC

**Inputs:**
- `StateEstimateMsg`
- `MissionStatusMsg`
- `NavStatusMsg`
- `BatteryStatusMsg`
- `SystemHealthMsg`

**Outputs:**
- `TelemetryPacketMsg` — formatted, prioritized outbound data for ground control

**Goal:**
To encode and transmit critical state, status, and health data to an offboard ground station or operator interface at configurable rates and priorities, supporting situational awareness and debugging.

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
