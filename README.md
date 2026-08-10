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

## Getting Started

### 1. Clone the Repository

Clone the repository together with its submodules:

```bash
git clone --recurse-submodules https://github.com/DEEP-UNDER-ROV/Ardupilot-SUPRI.git
cd Ardupilot-SUPRI
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
origin → https://github.com/DEEP-UNDER-ROV/Ardupilot-SUPRI.git
```
