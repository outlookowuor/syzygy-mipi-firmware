# SYZYGY-MIPI Firmware — `implement_i2c_bridge` branch

Firmware for TinyVision's SYZYGY-MIPI adapter (RP2040-based RP2350b board)
— a compact firmware collection for testing and running I²C bridge/slave examples
for TinyVision's TinyClunx33 / Syzygy-MIPI adapter hardware.

This branch focuses on I²C bridging and multiple I²C slave examples:
- `i2c_pio` — PIO driven I²C / bit-banged / helper utilities
- `i2c_slave` — single I²C slave demo
- `i2c_slave_multi` — multi-slave examples and tests

The adapter hardware and design are documented by TinyVision: see the syzygy MIPI adapter repo. :contentReference[oaicite:0]{index=0}

---

## Table of contents

- [Status](#status)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Getting the code](#getting-the-code)
- [Build (Raspberry Pi Pico / RP2040 SDK)](#build-raspberry-pi-pico--rp2040-sdk)
- [Flash / Deploy](#flash--deploy)
- [Wiring / Hardware notes](#wiring--hardware-notes)
- [Examples & usage](#examples--usage)
- [Testing & troubleshooting](#testing--troubleshooting)
- [Contribute](#contribute)
- [Credits & License](#credits--license)

---

## Status

Work in progress. The `implement_i2c_bridge` branch contains the latest changes and experimental code for bridging/I²C slave behavior. See the repository root for examples and folder layout. :contentReference[oaicite:1]{index=1}

---

## Features

- RP2040-targeted firmware (C + CMake + Pico SDK).
- PIO-based I²C helpers for non-standard timing or multi-bus scenarios.
- Example single I²C slave implementation (useful for testing host controllers).
- Multi-slave demo to exercise address collisions and multiplexing scenarios.
- Build system uses Raspberry Pi Pico SDK import (pico_sdk_import.cmake included). :contentReference[oaicite:2]{index=2}

---

## Prerequisites

On your host machine (Linux/macOS/Windows WSL), install the standard RP2040 toolchain and build tools:

- Git
- CMake (>= 3.13)
- Ninja (recommended) or make
- GCC arm-none-eabi toolchain (for building `.elf`)
- Raspberry Pi Pico/RP2040 SDK (the repo includes `pico_sdk_import.cmake` to point to your local SDK)
- `picotool` (optional, for command-line flashing), or the drag-and-drop UF2 method
- A USB cable to connect the RP2350-based board (or programmer like Picoprobe/OpenOCD if using SWD)

If you haven't set up the Pico toolchain before, the official Raspberry Pi Pico documentation is an excellent guide for installing the SDK and toolchain.

> Note: This repo includes `pico_sdk_import.cmake` to help the build system find your Pico SDK. Adjust `PICO_SDK_PATH` or set the `PICO_SDK_PATH` environment variable if your SDK is in a custom location. :contentReference[oaicite:3]{index=3}

---

## Getting the code

Clone the repo and check out the branch:

```bash
git clone https://github.com/outlookowuor/syzygy-mipi-firmware.git
cd syzygy-mipi-firmware
git checkout implement_i2c_bridge
```


## Build (Raspberry Pi Pico / RP2040 SDK)

This repository uses CMake and the Pico SDK. The build pattern below is the standard Pico workflow.

From the repo root:

```bash 
# create a build directory
cmake -S . -B build -G Ninja
cmake --build build -j
```

If your SDK is not in a standard location, set PICO_SDK_PATH on the environment before running cmake:

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -S . -B build -G Ninja
cmake --build build -j
```

After a successful build you will find build artifacts in build/. Depending on the folder and example you built, the generated .uf2 (for drag and drop flashing) or .elf/.bin will be available in build/ subdirectories.

```bash
If you prefer make:

mkdir -p build && cd build
cmake .. 
make -j
```


## Flash / Deploy

There are two common ways to flash an RP2040 board:

# 1) UF2 drag-and-drop (recommended for quick testing)

- Hold the BOOTSEL button on the RP2040 board while plugging it into USB (board enumerates as RPI-RP2).
- Copy the .uf2 file from build/ to the mounted drive.

# 2) picotool (command line)

- If you have picotool installed:

```bash
picotool load build/<target>.uf2
```

# 3) SWD / OpenOCD (advanced)

If you use a Picoprobe or external debugger, use your usual OpenOCD or probe workflow to flash the .elf or .bin.

## Wiring / Hardware notes

This repo targets the TinyVision Syzygy-MIPI adapter (RP2350b board). The adapter's hardware repo and specs are available upstream — review those docs for exact header pinouts and mechanical info. 
GitHub

General tips:

- Keep I²C pull-ups present and sized appropriately (typically 2.2k–10k depending on bus speed).
- If using multiple I²C slaves on the same bus, ensure unique 7-bit addresses or use the i2c_slave_multi example utilities to test address handling.
- For PIO based bit-banged I²C, watch bus timing — long wires, slow pull-ups, and capacitance may require slower bus speeds.

If you want wiring diagrams added to this README, tell me which adapter revision and which host board (Raspberry Pi, MCU, USB-I²C adapter) you plan to use and I’ll draft diagrams.