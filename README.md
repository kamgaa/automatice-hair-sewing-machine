### 

# Automatic Hair Sewing Machine (AHSM)

> **THIS project is written by @kamgaa (SeoulTECH)**
> 
> 
> Repository for maintaining firmware code for the **Automatic Hair Sewing Machine** of **RSM Global**.
> 

---

## Table of Contents

- [Before You Start (Prerequisites)]
- [Project Overview]
- [Hardware]
- [Toolchain]
- [Repo Layout]
- [Quick Start (STM32CubeIDE)]
- [Build & Flash]
- [Configuration via .ioc]
- [Debugging]
- [Coding Guidelines]
- [Troubleshooting]
- [Contributing]
- [License]

---

## Before You Start (Prerequisites)

1. **Install STM32CubeIDE** (latest recommended).
2. **Debugger:** Switch from **ST-Link** to **J-Link** (J-Link generally offers more robust features and faster flashing).
3. (Optional) Install **ST-LINK Utility** or **J-Link Commander** for standalone flashing.

> The repository contains a standard STM32CubeIDE project with Core/, Drivers/, .ioc (CubeMX config), and linker scripts for STM32F446ZETx (STM32F446ZETX_FLASH.ld, STM32F446ZETX_RAM.ld). GitHub
> 

---

## Project Overview

This firmware controls the **Automatic Hair Sewing Machine (AHSM)** actuators and I/O for RSM Global.

Core goals (tentative, to be refined as features land):

- Deterministic real-time control loop for the sewing mechanism
- Motor/actuator control (timers/PWM)
- Safety interlocks & fault monitoring (limits, E-stop, sensor sanity checks)
- UART/USB logging for diagnostics
- Modular HAL setup via STM32CubeMX (`.ioc`)

> NOTE: As of now, the repo looks like a freshly generated CubeMX project; feature modules will be documented here as they’re added.
> 

---

## Hardware

- **MCU:** STMicroelectronics **STM32F446ZETx** (Cortex-M4F @180 MHz)
- **Board:** Custom AHSM controller (or Nucleo/Discovery for bring-up)
- **Power:** Per board spec
- **Debug:** J-Link (recommended), ST-Link (fallback)

> The target part is indicated by the presence of STM32F446ZETX_*.ld linker scripts. GitHub
> 

---

## Toolchain

- **STM32CubeIDE** (includes GCC for Arm, CubeMX, and debug tooling)
- **STM32Cube HAL** (in `Drivers/`)
- **J-Link** tools (if using J-Link)

---

## Repo Layout

```
automatice-hair-sewing-machine/
├─ Core/            # Application code (Src/ & Inc/ expected)
├─ Drivers/         # STM32 HAL & CMSIS
├─ Debug/           # IDE-generated debug artifacts (may be local)
├─ .settings/       # IDE settings
├─ AHSM_0317.ioc    # CubeMX configuration (pinout/clock/middleware)
├─ AHSM_0317 Debug.launch  # IDE debug launch config
├─ STM32F446ZETX_FLASH.ld  # Linker (Flash)
├─ STM32F446ZETX_RAM.ld    # Linker (RAM)
├─ .cproject, .project, .mxproject  # IDE metadata

```

> Folder/file names confirmed from the repo root. GitHub
> 

---

## Quick Start (STM32CubeIDE)

1. **Clone** the repository:
    
    ```bash
    git clone https://github.com/kamgaa/automatice-hair-sewing-machine.git
    
    ```
    
2. **Open** STM32CubeIDE → **File ▸ Open Projects from File System...** → select the cloned folder.
3. The project should import automatically (CubeIDE recognizes `.project`/`.cproject`).

---

## Build & Flash

1. Select **Build Configuration**: *Debug* or *Release*.
2. **Build**: Right-click project → **Build Project**.
3. **Connect** your J-Link (or ST-Link) to the target board.
4. **Flash/Debug**:
    - With J-Link: use the provided **`AHSM_0317 Debug.launch`** or create a **GDB SEGGER J-Link Debugging** configuration, target **STM32F446ZE**.
    - With ST-Link: use **Ac6 STM32 Debugging** configuration for the same target.

---

## Configuration via `.ioc`

- Open **`AHSM_0317.ioc`** in STM32CubeIDE to view/modify:
    - **Clock tree** (HSE/HSI, PLL settings)
    - **Peripherals** (TIMx, UARTx, GPIO, DMA, etc.)
    - **Middleware** (if used)
- After changes, **Generate Code** to sync `Core/` and `Drivers/` sources.

> The .ioc file lives at repo root and is named AHSM_0317.ioc. GitHub
> 

---

## Debugging

- Prefer **J-Link** for faster downloads and RTT/SWO features.
- If breakpoints are missed, check:
    - Optimization level (use `Og` for debug)
    - Clock configuration (wrong clocks → SysTick timing issues)
    - Correct reset strategy in your *Debug Configuration* (connect under reset if needed)

---

## Coding Guidelines

- **C / C99**, follow STM32Cube HAL usage patterns.
- Keep app logic in `Core/Src/` and headers in `Core/Inc/`.
- Isolate hardware access to small modules (e.g., `motor.c`, `sensors.c`, `safety.c`).
- Use **`printf` retargeting** (UART/ITM) for logs; guard with `NDEBUG` for release.

---

## Troubleshooting

- **Project won’t build**:
    - Ensure you imported as an **existing STM32 project**, not just as files.
    - Right-click project → **Properties ▸ C/C++ Build ▸ Settings** and verify MCU/linker set to **STM32F446ZE**.
- **Cannot connect to target**:
    - Power and NRST lines; try “Connect Under Reset”.
    - J-Link firmware up to date; try lowering SWD speed (e.g., 400–1000 kHz).
- **Peripherals not running**:
    - Re-generate from `.ioc` to ensure HAL init code matches your configuration.

---

## Contributing

1. Create a feature branch: `feat/<short-desc>`
2. Run a clean build; test on hardware if possible.
3. Open a PR with a short demo note (what pins/timers were used, expected behavior).

---

## License

If this is proprietary to **RSM Global**, add a short proprietary notice here.

If you intend to open source, add a proper `LICENSE` file (e.g., MIT/BSD-3-Clause).

---

### Notes for Maintainers

- Consider committing an initial **README hardware map** (GPIOs, timers, UART pins) exported from the `.ioc` so others can wire the same setup.
- Add a short **CHANGELOG.md** as features mature (motor control, sensor feedback, safety states, communication protocol, etc.).
