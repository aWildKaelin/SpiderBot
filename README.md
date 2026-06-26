#  RP2040 Spider Bot

  
This project aims to create a RP2040-base quadrupedal walker based on MG90s servomotors.

Currently, the project is being developed around the Pico W board, with the intention of creating a custom PCB once development ends.

**This project is currently under active development.** The gait generation system and communication interface have not yet been implemented.

See `NOTEPAD.txt` for the complete status report of the codebase, as well as an explanation for how its structured.

## Setup

This project was constructed using `Visual Studio Code` and the `Raspberry Pi Pico` extension.

The `.vscode` folder is intentionally not included in the repository because it contains platform-specific configuration, which differs between Windows and Linux.

To open the project in Visual Studio Code, create a new Pico project using the extension, then replace the generated project files with the contents of this repository while keeping the generated `.vscode` directory.

## Third-party code

This project includes or uses `pico-ssd1306`, which is licensed under the BSD 3-Clause License. See the license file in the `pico-ssd1306` directory for details.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
