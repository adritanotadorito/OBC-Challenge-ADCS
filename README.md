# OBC Challenge – ADCS Debugging & Integration

## 📌 Overview
This repository contains my submission for the **CubeSat On-Board Computer (OBC) Challenge**.  
The task involved:
1. Fixing bugs in the provided ADCS (Attitude Determination and Control System) C++ source files.  
2. Integrating the corrected code into NASA’s **cFS** (Core Flight System) and the **42 simulator**, with telemetry flowing through the provided Python bridge.  
3. Demonstrating **fault injection** by deliberately introducing errors that breach mission thresholds.  

---

## 🛠️ Bug Fixes

###  `microcontroller_wrapper_c.cpp`
**Issue:** Code incorrectly referenced nested types (e.g. `Microcontroller::SensorData`) even though `SensorData` and `ActuatorCommands` were standalone structs.  
**Fix:** Removed the `Microcontroller::` prefix so the compiler could resolve the types.  
```cpp
// ❌ Before
auto sensorData = Microcontroller::SensorData{ ... };
Microcontroller::ActuatorCommands actuators;

// ✅ After
auto sensorData = SensorData{ ... };
ActuatorCommands actuators;
```
This allowed the wrapper to compile cleanly and correctly forward data between cFS (C) and the ADCS controller (C++).

### `adcs_controller.cpp`
- No new manual fixes required; the provided file already contained corrected logic.  
- Verified that PID initialization, array handling, and memory functions compile without error.  
- Rebuilt successfully with `-fPIC` to allow linking into a shared library.  

### General

Verified all source files (.cpp/.h) now compile without errors.

Ensured no core logic was changed — only bug fixes and logical corrections.

## 🔗 Integration Steps
After fixing the bugs, the corrected sources were compiled into a shared library (`libadcs_controllers.so`) and integrated into the existing cFS + 42 simulator environment.

**Commands used (inside Docker container):**

```bash
cd /home/osk/files

# Clean up old objects
rm -f *.o

# Compile with -fPIC for shared library compatibility
g++ -c -fPIC adcs_controller.cpp microcontroller.cpp microcontroller_wrapper_c.cpp -I.

# Link into shared library
g++ -shared -o ../libadcs_controllers.so adcs_controller.o microcontroller.o microcontroller_wrapper_c.o

# Run simulation
cd ..
./start_all.sh
```

**Telemetry output (from `telemetry_bridge.py`):**

0s Alt: 360.00km Drop: 0.00km Status: NORMAL <br>
1s Alt: 359.99km Drop: 0.01km Status: NORMAL <br>
2s Alt: 359.99km Drop: 0.01km Status: NORMAL <br>
3s Alt: 359.98km Drop: 0.02km Status: NORMAL <br>
...


This confirms that corrected ADCS outputs are linked into the Python telemetry script.
