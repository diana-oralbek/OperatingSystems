# Mars Rover Exploration System

A simulated Mars rover control system built with FreeRTOS, running on a POSIX port (Linux/macOS/Windows).

## Tasks

| Task | Priority | Period | Responsibility |
|------|----------|--------|----------------|
| MainComputer | 7 (highest) | 1000 ms | Mission orchestrator — reads system events and issues motor commands |
| Motor | 6 | 50 ms | Executes movement commands, updates dead-reckoning odometry |
| Comms | 5 | 500 ms | Simulates Earth uplink/downlink, relays status reports, detects signal loss |
| SelfMonitor | 4 | 500 ms | Watches heartbeats of all other tasks and triggers recovery on fault |
| Sensor | 3 | 100 ms | Simulates LIDAR distance, temperature, and altitude readings |
| Navigation | 2 | 200 ms | Records waypoints into a ring buffer, raises obstacle events |
| Power | 1 (lowest) | 1000 ms | Estimates power budget and suspends Navigation when budget is exceeded |

## Fault Tolerance

SelfMonitor checks a heartbeat table updated by every task each cycle. If a task misses its heartbeat for more than 2000 ms it is considered frozen. The Sensor task is automatically deleted and recreated on fault; other tasks log warnings for MainComputer to respond to.

---

## Installation & Setup

### For macOS users

1. **Install Xcode Command Line Tools** (provides GCC and make):
   ```bash
   xcode-select --install
   ```

2. **Install Python 3** (if not already installed) previously:
   ```bash
   brew install python
   ```
   Or download from [python.org](https://www.python.org/downloads/).

3. **Install Python dependencies**:
   ```bash
   pip3 install -r requirements.txt
   ```

---

### Windows

Windows does not natively support POSIX threads, so you will need **WSL (Windows Subsystem for Linux)**.

1. **Install WSL** (run in PowerShell as Administrator):
   ```powershell
   wsl --install
   ```
   Restart your computer when prompted. This installs Ubuntu by default.

2. **Inside WSL**, install GCC, make, and Python:
   ```bash
   sudo apt update
   sudo apt install gcc make python3 python3-pip
   ```

3. **Clone or copy the project into WSL**, then install Python dependencies:
   ```bash
   pip3 install -r requirements.txt
   ```

> **Note:** Run all build and Python commands from within the WSL terminal(Wsl to run the terminal), not from the standard Windows Command Prompt or PowerShell.

---

## Building & Running

```bash
make              # compile the rover binary
python3 dashboard.py   # launch the dashboard (opens at http://localhost:8050)
```

Use `make clean` to remove build artifacts.

The program boots, creates all tasks, seeds the RNG, then starts the FreeRTOS scheduler. Console output is prefixed by task name (e.g. `[Motor]`, `[Sensor]`).

---

## Project Structure

```
OperatingSystems/
├── src/
│   ├── main.c              # Entry point — IPC primitives, task creation, self-tests
│   ├── system.h / system.c # Shared types, handles, queues, mutexes, event group
│   ├── main_computer.c/h   # Mission orchestration — pendingTurn state machine
│   ├── motor.c/h           # Motor control, dead-reckoning odometry, obstacle detection
│   ├── sensor.c/h          # Drifting sensor simulation (temp/LIDAR/altitude)
│   ├── navigation.c/h      # 50-slot ring buffer waypoint map
│   ├── comms.c/h           # Earth comms — TX telemetry, RX commands, signal loss
│   ├── power.c/h           # Per-task power estimation, Navigation suspension
│   ├── self_monitor.c/h    # Heartbeat watchdog, vTaskDelete+xTaskCreate recovery
│   ├── mct.c/h             # Mission Control Terminal — interactive CLI + fault injection
│   ├── watchdog.c/h        # 8s one-shot FreeRTOS timer → EVENT_EMERGENCY_STOP
│   └── FreeRTOSConfig.h    # Tick rate, stack sizes, heap config
├── FreeRTOS-Kernel/        # FreeRTOS POSIX/GCC port
├── dashboard.py            # Python Dash GUI — subprocess pipe, live charts, route map
├── requirements.txt        # Python dependencies
├── Makefile                # Build system
└── README.md               # This file
```
