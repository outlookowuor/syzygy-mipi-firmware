# 🚀 Firmware for TinyVision's SYZYGY-MIPI Adapter

This repository contains the firmware and gateware necessary to operate the TinyVision SYZYGY-MIPI Adapter. This adapter acts as a bridge, allowing a SYZYGY-compliant FPGA carrier board (like the TinyClunx33) to interface with 3 standard MIPI CSI-2 camera modules.


## Table of Contents

- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Toolchain Setup](#toolchain-setup)
- [Building the Firmware](#building-the-firmware)
- [Flashing the Board](#flashing-the-board)
- [Module Overview](#module-overview)
  - Main Menu
  - Syzygy DNA
  - GPIO Expander
  - Clock Programmer
  - I²C Muxer & I²C Bridging
- [Example Usage](#example-usage)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [Credits](#credits)
- [License](#license)

---

## ✨ Features

The primary features and current development goals include:

- **SYZYGY Interoperability:** Providing the necessary logic to communicate with a host FPGA carrier board over the **SYZYGY high-speed interface.**

- **SYZYGY to 3x MIPI devices:** Provide multiplexed access to 3 MIPI devices via the syzygy connector. This includes:
    - Access to 2x MIPI device GPIOs (one clock programmable)
    - I2C connectivity

- **Hardware Base:** Specifically targets the adapter board based on the **RP2350b** chip.

- **Target Device:** Designed to work seamlessly with TinyVision's TinyClunx33 board.

---

## Hardware Requirements

| Component | Description |
|------------|-------------|
| **RP2350 Board** | e.g. TinyVision SYZYGY-MIPI Adapter (RP2350b) |
| **SYZYGY Peripheral** | e.g. MIPI sensor or expansion module |
| **USB Cable** | For power, flashing, and serial console |
| **Optional Tools** | Logic analyzer (for bus debugging), Picoprobe or OpenOCD (for SWD debugging) |


### Electrical Notes
- Ensure proper **I²C pull-ups** on SDA/SCL lines (typically 2.2k–10kΩ).
- When using multiple I²C slaves, confirm unique 7-bit addresses.
- Keep wiring short for stable high-speed I²C operation.

---

## Toolchain Setup

To build this firmware, install the following on your host system (Linux/macOS/Windows):

- `git`
- `cmake` ≥ 3.13
- `ninja` *(recommended)*
- `gcc-arm-none-eabi`
- `picotool` *(for flashing via USB)*
- **Raspberry Pi Pico SDK**

Clone or set your Pico SDK path:

```bash
git clone https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
export PICO_SDK_PATH=~/pico-sdk
```

---


## Building the Firmware

Clone and build the firmware:

```bash
git clone https://github.com/outlookowuor/syzygy-mipi-firmware.git
cd syzygy-mipi-firmware
mkdir build && cd build
cmake .. -G Ninja
ninja
```

If successful, you’ll find compiled `.uf2`, `.bin`, and `.elf` files in the `build/` directory and its subfolders.

---


## Flashing the Board

### Option 1: USB (Drag-and-Drop)
1. Hold the **BOOTSEL** button on the board.
2. Plug it into your PC via USB — it will appear as a mass storage device (`RPI-RP2`).
3. Copy the generated `.uf2` file onto it.
4. The board will reboot automatically.

### Option 2: Using `picotool`
If you prefer CLI flashing:

```bash
picotool load build/syzygy_mipi.uf2
```

---

## Module Overview
Below is a summary of the major software modules 

### Main Menu
This module implements a simply menu accessible through UART on GPIO pins 16/17.
The interactive menu allows the user to:
- set i2c addresses for the 
    - i2c-multiplexer
    - i2c Bridge
    - GPIO expander
    - Clock Generator
- view the settings for the above
- save the settings to flash
- view SYZYGY settings


### SYZYGY DNA

SYZYGY DNA I²C slave functionality is provided through the Host SYZYGY DNA I²C connection. 
Only read-only functionality is provided for the moment.
Programming will be done with the firmware on firmware updates.


### I²C Muxer & I²C Bridging

At any given time, the carrier/host should be able to communicate with any one of the 3 MIPI devices via i2c, through the adaptor's Host I2C pins.

The I²C "muxer" allows the user to select which of 3 devices is the active one at any given time.

The I²C "bridge" is simply an address accessible to the host that bridges any communication with it to the selected MIPI device.



### GPIO Expander

Strictly speaking, this module is different from a traditional GPIO expander. Each of the 3 MIPI devices has a GPIO that can be controlled by the host through Host GPIO pin.

This module simply presents an I²C interface used to:
- Set the active GPIO as either input or output
- Select a GPIO (MIPI device) as "active" at any given time

The host, through its Host GPIO can either read the value of the "active" GPIO, if it has been set as input, or write high/low to the active MIPI GPIO.

Examples of how to do this are documented at the top of the source code.


### Clock Programmer

The MIPI devices have a second GPIO that has clock-programmable capability connected to the adaptor.

This module presents an I²C interface used to:
- Set the frequency & duty of the clocks
- Select the GPIO (MIPI device) that is "active"

Examples of how to do this are documented at the top of the source code.

---

## Example Usage
1. Connect the adapter board to the host FPGA via SYZYGY interface; connect the MIPI sensor(s).
2. Flash the firmware as described above.
3. On serial console, observe initialisation logs: clock programming, I²C mux selection, sensor resets, link negotiation.
4. Use your host‑FPGA design (or example code) to start MIPI capture; verify output frames.

---
---

## Troubleshooting

| Symptom | Possible Cause | Resolution |
|----------|----------------|-------------|
| Board not detected as `RPI-RP2` | Not in BOOTSEL mode | Hold BOOTSEL while plugging in USB |
| Flash fails via picotool | Missing udev permissions or device busy | Check `lsusb`, run `sudo dmesg`, or use drag-and-drop UF2 |
| I²C not responding | Wrong address or missing pull-ups | Verify wiring, check with `i2cdetect`, ensure pull-ups present |
| Random resets | Power instability | Use a stable 5 V USB source or powered hub |

---

## Contributing

Contributions are welcome!  
Please:
1. Fork the repository.
2. Create a branch from `main`.
3. Submit a pull request with clear description and testing steps.

Follow these guidelines:
- Keep commits atomic and descriptive.
- Add comments for non-trivial PIO or timing-sensitive code.
- Include example output or waveform screenshots when possible.

---

## Credits

- **Firmware development:** [Bernard Adongo](https://github.com/outlookowuor) and contributors  
- **Hardware:** TinyVision.ai — [TinyClunx33 / SYZYGY-MIPI Adapter](https://tinyclunx33.tinyvision.ai/)
- **SDK:** [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)

---

## License

This project is open source.  
Unless otherwise noted, it is licensed under the **MIT License** — see [`LICENSE`](LICENSE) for details.
