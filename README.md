Mars Rover Exploration System
A simulated Mars rover control system built with FreeRTOS, demonstrating real-time operating system concepts including multitasking, inter-task communication, synchronization primitives, and fault tolerance.

Overview
The system models a rover operating autonomously on Mars with seven concurrent FreeRTOS tasks that communicate via queues, mutexes, and event groups. Each task mirrors a real subsystem found in planetary rover architectures.

Simulated Sensors
Readings drift realistically each cycle within Mars-plausible ranges:

Temperature: −80 °C to −50 °C (fixed-point, Celsius × 100)
LIDAR distance: 20 cm to 500 cm (front obstacle)
Altitude: −200 cm to +200 cm (terrain elevation)
Odometry
The Motor task maintains dead-reckoning position in a 2-D grid (x/y in cm, heading in 90° increments: N/E/S/W). Simulated terrain noise (±10 cm per move) is added to each forward/backward command.

Building
Requires GCC and pthreads (Linux/macOS or WSL on Windows).


make
This compiles the FreeRTOS POSIX port together with all rover source files into the mars_rover binary.

Clean

make clean
Running

make run
# or
./mars_rover
The program boots, creates all tasks, seeds the RNG, then starts the FreeRTOS scheduler. Console output is prefixed by task name (e.g. [Motor], [Sensor]).

Project Structure
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

Key FreeRTOS Concepts Demonstrated
Preemptive scheduling with fixed priorities
Task-to-task communication via queues
Mutual exclusion with binary semaphores/mutexes
Event-driven coordination with event groups
Task suspension and resumption (Power ↔ Navigation)
Dynamic task deletion and recreation (SelfMonitor → Sensor)
Heartbeat-based watchdog monitoring
Stack overflow and malloc-failure hooks
