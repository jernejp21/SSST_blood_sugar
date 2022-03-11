# SSST_blood_sugar

Wearable device for measuring blood sugar wihtout analysing the blood or pircing th skin.

## SDK Installation

[Vido instructions](https://youtube.com/playlist?list=PLx_tBuQ_KSqEt7NK-H7Lu78lT2OijwIMl)

Windows:
1. Install nRF Command Line Tools ([Download](https://bit.ly/2YgBGC5))
2. Install nRF Connect SDK through the Toolchain Manager in nRF Connect for Desktop ([Download](https://bit.ly/39Tm3my))
3. Install Visual Studio Code ([Download](https://code.visualstudio.com/Download))
4. **Make sure to install SDK version 1.9.0!**

## Development setup

1. After cloning git repository, open `Firmware.code-workspace` with VS Code.
2. Go to nRF Connect and create *build configuration*. Under APPLICATIONS add build configuration to Firmware.
   - Board: `nrf52840dk_nrf52840`
   - Build directory name: `build`
   - Enable debug options

3. Check `Pics/nRF52840DK.jpg` for how to connect power and debugger to development kit.

For API and how to, chekc [Nordic's developer website](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.0/nrf/index.html).

### Git commit prefixes
When commiting, use prefix for easier changelog generation.

Good example: `<prefix>: <Issue title>`

Bad example: `Fixed this and this with this commit`

Prefixes:
- `enh` for enhancement (new feature)
- `bug` for bug fixes
- `hw` for HW update
- `ref` for code refactoring

