
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
  The system is organized into discrete Computer Software Components (CSCs) to modularize and clearly define all system responsibilities.
- **Hardware Abstraction Layer (HAL):**  
  Provides portability between different hardware platforms.
- **Simulation with Gazebo:**  
  Allows for rapid prototyping and testing in a simulated environment.
- **Unit Testing:**  
  Leverages Google Test for continuous, automated unit testing.
- **Machine Learning:**  
  Planned phase for research and integration of learning-based control algorithms.


## CSCs

AutoTross is designed around modular **Computer Software Components (CSCs)**, which represent the major functional units of the UAV system. These CSCs communicate with each other using an **event-driven messaging system** to ensure decoupled and scalable interactions.

TBD List

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


- **RBuilding:**
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

### Simulation

Gazebo is used for simulation. Detailed instructions for setting up simulation environments and running flight scenarios will be provided as development progresses.


## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
