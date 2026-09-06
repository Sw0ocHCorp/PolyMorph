# PolyMorph

> **Documentation** — architecture, mathematics and control pipelines: see [`docs/`](docs/README.md).
> Build it with `cargo install mdbook` then `cd docs && mdbook serve --open`.


PolyMorph is an Open Source Robotic Rust ecosystem.
It is composed of:
* Remote Controller
* Companion Software
* Simulation Flight Controller Software using Godot as Physics Engine

For the begining of the project I build my own encoding system for the communication between all the softwares:

The SOF of ALL the Frames is "abcd"

## FEATURE DATA ID
| Data Label                    | Data ID   | Data                                          |
| :-------------:               | :-------: | :-------------:                               |
| Command                       | 0001      | Single value (Arm `0a` DisArm `0f`)           |
| Teleoperation                 | 0002      | Single value (Arm `0a` DisArm `0f`)           |
| GNSS Positioning              | 0003      | Single value (Arm `0a` DisArm `0f`)           |
| Internal Perception           | 0004      | Single value (Arm `0a` DisArm `0f`)           |
| Lidar Scan                    | 0005      | Single value (Arm `0a` DisArm `0f`)           |
| Camera Vision                 | 0006      | Single value (Arm `0a` DisArm `0f`)           |
Thoses are the IDs of a data chunk. 
A Data chunk is a collection of potentials measurement, data, etc... relative to a feature

## Teleoperation Data ID
| Data Label                    | Data ID   | Data                                          |
| :-------------:               | :-------: | :-------------:                               |
| Arming/Disarm                 | 000a      | Single value (Arm `0a` DisArm `0f`)           |
| Joystick                      | 000b      | 1 Value / Axis (2 Joystick, 4 Axis)           |

## GNSS Positioning Data ID
| Data Label                    | Data ID   | Data                                          |
| :-------------:               | :-------: | :-------------:                               |
| GPS Lat/Lon Mesurement        | 0012      | Lat/Lon/                                      |
| GPS Speed Heading Mesurement  | 0013      | Speed()/Heading()                             |
| Lidar Measurements            | 0014      | Specific Frame with the LiDAR data            |

## Internal Perception Data ID
| Data Label                    | Data ID   | Data                                          |
| :-------------:               | :-------: | :-------------:                               |
| Drone Speed Setpoint          | 000c      | 1 Value / Axis (3D Vector)                    |
| Drone Angles Setpoint         | 000d      | For the begining 1 Value                      |
| Servos Angles Setpoint        | 000e      | 1 Value / Servos                              |
| Drone Speed Mesurement        | 000f      | 1 Value Axis (Sensor Fusion, 2 IMUs: 3 Axis)  |
| Drone Gyro Mesurement         | 0010      | 1 Value Axis (Sensor Fusion, 2 IMUs: 3 Axis)  |
| Altitude Mesurement           | 0011      | 1 Value (Sensor Fusion, 2 Barometers: 1 Value)|
| Battery Mesurement            | 0015      | 1 Value (Remain Energy)                       |

### Lidar Specific Frame
| Data Label                    | Data ID   | Data                                                  |
| :-------------:               | :-------: | :-------------:                                       |
| Lidar measurements range      | 000a      | 2 angles representing the angle range of the lidar    |
| Lidar measurements            | 000b      | LiDAR measurements (distance sensor <-> object)       |
