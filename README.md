# ArduSub SUPRI Version
Modified ArduPilot firmware for **SUPRI (Submersible Underwater Platform for Research and Inspection)**, a 6-DOF underwater ROV developed for underwater inspection and mapping.
This repository contains custom modifications to the ArduPilot firmware, including the implementation of a **Sliding Mode Controller (SMC)** for the ROV attitude control system and custom thruster configuration.
## Repository Structure
```text
Ardupilot-SUPRI/
├── ArduSub/
├── libraries/
├── Tools/
├── modules/
├── waf
├── .gitmodules
└── README.md
```
The repository follows the standard ArduPilot source tree with custom SUPRI modifications.

## Code Modifications

This section describes the modifications made to the ArduSub codebase for the SUPRI ROV.

### 1. Thruster Configuration

A custom thruster configuration was added to `libraries/AP_Motors/AP_Motors6DOF.cpp` in the `AP_Motors6DOF::setup_motors()` function. The configuration is based on the calculated thruster allocation shown in the link below.

[Propulsion Design](https://itbdsti-my.sharepoint.com/:b:/r/personal/hilton_tnunay_itb_ac_id/Documents/Workspace/02_Education/03_Courses/2025-2026/S02/B-EL4091-TA/01_Projects/05_TA252601031_UW-Archaeology/02_TA252601031/05_Products/Propulsion%20Design.pdf?d=w4091da08cfda47149aaea5cf3e6d305f&csf=1&web=1&e=H5xXqI)

### 2. SMC Implementation

A Sliding Mode Controller (SMC) was implemented to replace the default PID rate controller in `libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.cpp`.

The controller is implemented in the `AC_AttitudeControl_Sub::rate_controller_run()` function. In the current implementation, the default PID controller is located at line 594, while the SMC implementation is located at line 608.

Several SMC-related declarations used by the controller are defined in `libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.h`.

The SMC is executed within the ArduSub rate-control loop and replaces the default PID controller for attitude rate control.

## Getting Started
### 1. Clone the Repository

Clone the repository together with its submodules:

```bash
git clone --recurse-submodules https://github.com/DEEP-UNDER-ROV/Ardusupri.git
cd Ardusupri
```

If the repository has already been cloned without submodules:

```bash
git submodule update --init --recursive
```

### 2. Set Up the Build Environment

ArduPilot requires a compatible Python environment and ARM embedded toolchain for firmware compilation.

For this version of the repository, use a Python version compatible with the included WAF build system.

Example:

```bash
source sitl_venv/bin/activate
```

### 3. Configure the Firmware

For a Pixhawk 2.4.8 / FMUv3 target:

```bash
./waf configure --board fmuv3
```

### 4. Build

Build the firmware with:

```bash
./waf sub
```

The generated firmware will be available under:

```text
build/fmuv3/bin/
```

## Development Workflow
The official ArduPilot repository is configured as the upstream remote:

```text
upstream → https://github.com/ArduPilot/ardupilot.git
```

The SUPRI repository is configured as the origin remote:

```text
origin → https://github.com/DEEP-UNDER-ROV/Ardusupri.git
```
