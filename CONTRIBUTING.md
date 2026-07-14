# Contributing to MultiNav Pro+ BLE Module Firmware

Thank you for your interest in contributing! This document explains how to report issues, propose changes, and submit code.

## 📋 Ways to Contribute

- **Report bugs** — open a [GitHub Issue](https://github.com/RFOXiA/MultiNav-Pro-Long-Range-BLE-Module-Firmware-STM32WB07/issues) with steps to reproduce
- **Suggest features** — open an Issue describing the use case and proposed behavior
- **Improve documentation** — typo fixes, clarifications, and better examples are always welcome
- **Submit code** — bug fixes, driver improvements, new sensor support, BLE enhancements

## 🐛 Reporting Bugs

Please include:

1. **Firmware version** (release tag or commit hash)
2. **Hardware setup** — module revision, connected sensors/peripherals, power source
3. **Steps to reproduce** — exact sequence that triggers the problem
4. **Expected vs. actual behavior**
5. **Logs** — UART debug output if available (see `debug_log.h`)
6. **Companion app details** if relevant — RFOXiA Connect version, phone OS

## 🔀 Pull Request Process

1. **Fork** the repository and create a branch from `main`:
   ```
   git checkout -b fix/short-description
   ```
2. **Make your changes** — keep each PR focused on a single fix or feature
3. **Build and verify**:
   - Open the project in **STM32CubeIDE** (1.19.0+)
   - `Project → Clean Project`, then `Project → Build Project`
   - Confirm `Build Finished. 0 errors.`
   - Flash to real hardware and test the affected functionality (see README §3)
4. **Commit** with a clear message describing *what* and *why*
5. **Open a Pull Request** against `main` with:
   - A summary of the change
   - The hardware you tested on
   - Any related Issue numbers (`Fixes #123`)

## 🎨 Code Style

Follow the existing conventions in the codebase:

- **C11**, 2-space indentation, no tabs
- `snake_case` for functions and variables, `UPPER_CASE` for macros/constants
- One module per `.c`/`.h` pair — keep drivers self-contained (see `Core/Inc` / `Core/Src`)
- Prefer non-blocking, interrupt/DMA-driven patterns — avoid busy-wait loops in runtime paths
- Do not edit auto-generated CubeMX regions outside the `USER CODE BEGIN`/`END` markers
- Document non-obvious logic with brief comments; keep public APIs documented in headers

## ⚠️ Scope Notes

- **BLE stack, HAL, CMSIS** (`Drivers/`, `Middlewares/`) are vendor code — changes there are rarely accepted; report upstream issues to ST instead
- **Hardware design** is proprietary and out of scope for this repository
- Regenerating from the `.ioc` file can touch many files — if your change requires CubeMX regeneration, mention it explicitly in the PR

## 📜 License of Contributions

By submitting a contribution, you agree that it is licensed under the [Apache License 2.0](LICENSE), consistent with Section 5 of the license. No rights to the "RFOXiA" or "MultiNav Pro+" trademarks are granted or implied.

## 💬 Questions?

- Community & documentation: [RFOXiA Club](https://club.rfoxia.com/)
- Product page: [MultiNav Pro+ Premium BLE Module](https://rfoxia.com/premium-ble-module/)
