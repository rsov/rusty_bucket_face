# Rusty Bucket Face




# --- DOCS ---

ESP hal docs https://docs.esp-rs.org/esp-hal/esp-hal/0.16.1/esp32s3/esp_hal/


### Display

Waveshare 1.28 inch Touch LCD
Docs https://www.waveshare.com/wiki/1.28inch_Touch_LCD


| Type             | Data       |
| ---------------- | ---------- |
| Resolution       | 240x240px  |
| LCD controller   | GC9A01     |
| LCD interface    | 4-wire SPI |
| Touch controller | CST816S    |
| Touch interface  | I2C        |



Connection to Arduino Nano ESP32

| Display | Arduino | Color  | GPIO |
| ------- | ------- | ------ | ---- |
| VCC     | 3.3V    | Red    |      |
| GND     | GND     | Black  |      |
| MISO    | D12     | Blue   | 47   |
| MOSI    | D11     | Yellow | 38   |
| SCLK    | D13     | Orange | 48   |
| LCS_CS  | D10     | Green  | 21   |
| LCS_DC  | D7      | White  |      |
| LCS_RST | D8      | Purple |      |
| LCS_BL  | D9      | Brown  | 18   |
| TP_SDA  | A4      | Grey   |      |
| TP_SCL  | A5      | Blue   |      |
| TP_INT  | D3      | Yellow |      |
| TP_RST  | D4      | Purple |      |



# --- END DOCS ---
Auto generated stuff below

## Dev Containers
This repository offers Dev Containers supports for:
-  [VS Code Dev Containers](https://code.visualstudio.com/docs/remote/containers#_quick-start-open-an-existing-folder-in-a-container)
-  [GitHub Codespaces](https://docs.github.com/en/codespaces/developing-in-codespaces/creating-a-codespace)
> **Note**
>
> In [order to use GitHub Codespaces](https://github.com/features/codespaces#faq)
> the project needs to be published in a GitHub repository and the user needs
> to be part of the Codespaces beta or have the project under an organization.

If using VS Code or GitHub Codespaces, you can pull the image instead of building it
from the Dockerfile by selecting the `image` property instead of `build` in
`.devcontainer/devcontainer.json`. Further customization of the Dev Container can
be achieved, see [`.devcontainer.json` reference](https://code.visualstudio.com/docs/remote/devcontainerjson-reference).

When using Dev Containers, some tooling to facilitate building, flashing and
simulating in Wokwi is also added.
### Build
- Terminal approach:

    ```
    scripts/build.sh  [debug | release]
    ```
    > If no argument is passed, `release` will be used as default


-  UI approach:

    The default build task is already set to build the project, and it can be used
    in VS Code and GH Codespaces:
    - From the [Command Palette](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette) (`Ctrl-Shift-P` or `Cmd-Shift-P`) run the `Tasks: Run Build Task` command.
    - `Terminal`-> `Run Build Task` in the menu.
    - With `Ctrl-Shift-B` or `Cmd-Shift-B`.
    - From the [Command Palette](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette) (`Ctrl-Shift-P` or `Cmd-Shift-P`) run the `Tasks: Run Task` command and
    select `Build`.
    - From UI: Press `Build` on the left side of the Status Bar.

### Flash

> **Note**
>
> When using GitHub Codespaces, we need to make the ports
> public, [see instructions](https://docs.github.com/en/codespaces/developing-in-codespaces/forwarding-ports-in-your-codespace#sharing-a-port).

- Terminal approach:
  - Using `flash.sh` script:

    ```
    scripts/flash.sh [debug | release]
    ```
    > If no argument is passed, `release` will be used as default

- UI approach:
    - From the [Command Palette](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette) (`Ctrl-Shift-P` or `Cmd-Shift-P`) run the `Tasks: Run Task` command and
    select `Build & Flash`.
    - From UI: Press `Build & Flash` on the left side of the Status Bar.
- Any alternative flashing method from host machine.


### Wokwi Simulation

#### VS Code Dev Containers and GitHub Codespaces

The Dev Container includes the Wokwi Vs Code installed, hence you can simulate your built projects doing the following:
1. Press `F1`
2. Run `Wokwi: Start Simulator`

> **Note**
>
>  We assume that the project is built in `debug` mode, if you want to simulate projects in release, please update the `elf` and  `firmware` proprieties in `wokwi.toml`.

For more information and details on how to use the Wokwi extension, see [Getting Started] and [Debugging your code] Chapter of the Wokwi documentation.

[Getting Started]: https://docs.wokwi.com/vscode/getting-started
[Debugging your code]: https://docs.wokwi.com/vscode/debugging

> **Warning**
>
>  ESP32-C2 is not, yet, not supported in Wokwi.


