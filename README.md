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

## Building & Running

Requires GCC and pthreads (Linux/macOS or WSL on Windows).

```bash
make        # build
make clean  # remove build artifacts
python/python3 dashboard.py #to run the project 
The program boots, creates all tasks, seeds the RNG, then starts the FreeRTOS scheduler. Console output is prefixed by task name (e.g. [Motor], [Sensor]).

Project Structure
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
├── Makefile                # Build system
└── README.md               # Build and run instructions
