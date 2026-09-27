# System Architecture Diagram

```mermaid
graph TD
    subgraph Sensors Service ["drone-sensors.service (30Hz)"]
        VL53L1X_Down["VL53L1X Down (I2C: 0x30 / GPIO 17)"]
        VL53L1X_Front["VL53L1X Front (I2C: 0x31 / GPIO 27)"]
        PMW3901["PMW3901 (SPI CS: GPIO 8)"]
        SensProc["Sensor Loop (Filters & Deadbands)"]
        VL53L1X_Down --> SensProc
        VL53L1X_Front --> SensProc
        PMW3901 --> SensProc
    end

    subgraph Shared Memory ["POSIX Shared Memory (core.shared_memory_manager)"]
        SHM_SENS["drone_sensor_data<br/>(7x float64: alt, front, sx, sy, vx, vy, hb)"]
        SHM_HB["system_heartbeats<br/>(3x float64: sensors, inference, fc)"]
    end

    subgraph Inference Service ["drone-inference.service (100Hz)"]
        PID["FlightController (PID Throttle)"]
        InfLoop["Inference / Control Logic"]
        PID --> InfLoop
    end

    subgraph Flight Controller Service ["drone-fc.service (100Hz)"]
        FCReal["FCReal (MSPy / MSP over Serial @ 115200)"]
    end

    subgraph UI & Monitoring ["dashboard.py / rich_ui.py"]
        TUI["Rich Terminal UI & Keyboard Input"]
    end

    SensProc -->|Write| SHM_SENS
    SensProc -->|Update HB[0]| SHM_HB

    SHM_SENS -->|Read Obs| InfLoop
    InfLoop -->|Update HB[1]| SHM_HB

    TUI -->|PUB 'tcp://127.0.0.1:5555'<br/>(disarmed/armed/ai)| InfLoop
    InfLoop -->|PUB 'tcp://127.0.0.1:5556'<br/>(RC: roll, pitch, throttle, yaw, aux1, aux2)| FCReal
    InfLoop -->|PUB 'tcp://127.0.0.1:5556'| TUI

    FCReal -->|Update HB[2]| SHM_HB
    SHM_HB -->|Read Status| TUI
    SHM_SENS -->|Read Telemetry| TUI
```
