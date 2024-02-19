## Dependencies
This project uses the Espressif [ESP-IDF version 5.1.1 for the ESP32S3](https://docs.espressif.com/projects/esp-idf/en/v5.1.1/esp32s3/get-started/index.html).

### Nix
Optionally you can use Nix to initialize a shell.

Firstly you require Nix to be installed, and to have the following optional features enabled (in `~/.config/nix/nix.conf`):
```
experimental-features = nix-command flakes
```

Then you can run in the top-level directory of this repository:
```bash
nix develop
```

All the required dependencies should now be loaded in this shell.

## How to build
```bash
rm ./sdkconfig
export SDKCONFIG_DEFAULTS="./sdkconfig.default"
idf.py clean
rm -rf build

idf.py menuconfig
# Now set all required options under Leddimmer

idf.py build
```

## How to flash
Connect the device over USB-C to your computer and run
```bash
idf.py flash
```

You can attach to the virtual terminal of the device by running
```bash
idf.py monitor
```

For more information to use this monitor, see the [ESP-idf documentation on monitor](https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32s3/api-guides/tools/idf-monitor.html).
