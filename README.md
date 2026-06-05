# Mars Rover Exploration System

A simulated Mars rover control system built with **FreeRTOS**, demonstrating core real-time OS concepts: multitasking, inter-task communication, synchronization, and fault tolerance.

---

## Table of Contents

1. [Overview](#overview)
2. [Project Structure](#project-structure)
3. [Architecture](#architecture)
   - [Tasks](#tasks)
   - [Inter-task Communication](#inter-task-communication)
   - [Fault Tolerance](#fault-tolerance)
   - [Simulated Sensors](#simulated-sensors)
   - [Odometry](#odometry)
4. [Build & Run](#build--run)
5. [FreeRTOS Concepts Demonstrated](#freertos-concepts-demonstrated)

---

## Overview

The system models a rover operating autonomously on Mars with **7 concurrent FreeRTOS tasks**. Each task represents a real rover subsystem and communicates with others through queues, mutexes, and event groups.

---

## Project Structure

```
OperatingSystems-main/
├── src/
│   ├── main.c              # Entry point — creates IPC primitives and tasks
│   ├── system.h / system.c # Shared types, handles, and helper functions
│   ├── main_computer.c/h   # Mission orchestration
│   ├── motor.c/h           # Motor control and odometry
│   ├── sensor.c/h          # Sensor simulation
│   ├── navigation.c/h      # Waypoint mapping
│   ├── comms.c/h           # Earth communications
│   ├── power.c/h           # Power budget management
│   ├── self_monitor.c/h    # Watchdog and task recovery
│   └── FreeRTOSConfig.h    # FreeRTOS configuration
└── FreeRTOS-Kernel/        # FreeRTOS source (POSIX/GCC port)
```

---

## Architecture

### Tasks

| Task | Priority | Period | Responsibility |
|------|----------|--------|----------------|
| MainComputer | 7 (highest) | 1000 ms | Mission orchestrator — reads system events and issues motor commands |
| Motor | 6 | 50 ms | Executes movement commands, updates dead-reckoning odometry |
| Comms | 5 | 500 ms | Simulates Earth uplink/downlink, relays status reports, detects signal loss |
| SelfMonitor | 4 | 500 ms | Watches heartbeats of all other tasks and triggers recovery on fault |
| Sensor | 3 | 100 ms | Simulates LIDAR distance, temperature, and altitude readings |
| Navigation | 2 | 200 ms | Records waypoints into a ring buffer, raises obstacle events |
| Power | 1 (lowest) | 1000 ms | Estimates power budget and suspends Navigation when budget is exceeded |

---

### Inter-task Communication

| Mechanism | Name | Purpose |
|-----------|------|---------|
| Queue | `xCommandQueue` | Motor commands from MainComputer / Earth |
| Queue | `xSensorQueue` | Sensor readings from Sensor → Navigation |
| Queue | `xStatusQueue` | Status reports from all tasks → Comms (to Earth) |
| Mutex | `xSensorDataMutex` | Protects shared sensor data buffer |
| Mutex | `xMotorMutex` | Serializes motor command execution |
| Mutex | `xNavMapMutex` | Protects the waypoint ring buffer |
| Mutex | `xOdometryMutex` | Protects rover position/heading data |
| Event Group | `xSystemEventGroup` | System-wide event flags (see below) |

**Event flags:**

| Flag | Trigger |
|------|---------|
| `EVENT_OBSTACLE_DETECTED` | Motor or Navigation detects object within 50 cm |
| `EVENT_LOW_POWER` | Power budget exceeds maximum |
| `EVENT_COMMS_LOST` | No Earth contact for 3000 ms |
| `EVENT_SENSOR_FAULT` | Sensor mutex timeout or heartbeat lost |
| `EVENT_MOTOR_FAULT` | Motor heartbeat lost |
| `EVENT_EMERGENCY_STOP` | Emergency stop commanded |
| `EVENT_MISSION_COMPLETE` | Mission objective reached |

---

### Fault Tolerance

The **SelfMonitor** task checks a heartbeat table that every other task updates each cycle. A task that misses its heartbeat for more than **2000 ms** is considered frozen.

| Task | Recovery Action |
|------|----------------|
| Sensor | Deleted and recreated automatically |
| Motor | `EVENT_MOTOR_FAULT` set, warning logged |
| Comms | `EVENT_COMMS_LOST` set, warning logged |
| Navigation | Warning logged (skipped if intentionally suspended by Power) |
| MainComputer | Warning logged |

---

### Simulated Sensors

Readings drift realistically each cycle within Mars-plausible ranges:

| Sensor | Range | Unit |
|--------|-------|------|
| Temperature | −80 to −50 | °C (stored as Celsius × 100) |
| LIDAR distance | 20 to 500 | cm (front obstacle) |
| Altitude | −200 to +200 | cm (terrain elevation) |

---

### Odometry

The Motor task maintains dead-reckoning position in a 2-D grid:

- **Position**: x/y in cm from origin
- **Heading**: 0° = N, 90° = E, 180° = S, 270° = W (90° increments)
- **Terrain noise**: ±10 cm random variation added per move
- **Distance from origin**: computed as Euclidean distance, reported after each move

---

## Build & Run

**Requirements:** GCC, pthreads (Linux/macOS or WSL on Windows)

```bash
# Build
make

# Run
make run

# Clean
make clean
```

Console output is prefixed by task name, e.g. `[Motor]`, `[Sensor]`, `[Comms]`.

---

## FreeRTOS Concepts Demonstrated

- Preemptive scheduling with fixed priorities
- Task-to-task communication via queues
- Mutual exclusion with mutexes
- Event-driven coordination with event groups
- Task suspension and resumption (Power ↔ Navigation)
- Dynamic task deletion and recreation (SelfMonitor → Sensor)
- Heartbeat-based watchdog monitoring
- Stack overflow and malloc-failure hooks
