# Source this to get `idf.py` in your shell:   source idf-env.sh
#
# GOTCHA this solves: ESP-IDF v5.5.3 for these projects lives in the SISTER rover
# project's .tools/, and its toolchain was installed to a PROJECT-LOCAL path, NOT the
# default ~/.espressif. So a bare `source .../export.sh` fails with:
#     ERROR: ESP-IDF Python virtual environment ".../.espressif/.../python" not found.
# Setting IDF_TOOLS_PATH to the project-local espressif dir before sourcing fixes it.
export IDF_TOOLS_PATH="/home/arzaan/PROJECTS/home-rover/.tools/espressif"
export IDF_PATH="/home/arzaan/PROJECTS/home-rover/.tools/esp-idf-v5.5.3"
source "$IDF_PATH/export.sh"

# ---- Proven flash flow on this WSL laptop ------------------------------------------
# The ESP32 board's USB-serial chip is a CH340 (hardware-id 1a86:7523). WSL doesn't see
# USB by default — attach it from Windows FIRST, then it shows up as /dev/ttyUSB0:
#
#   1) In Windows PowerShell (board plugged in):
#        usbipd list                                  # confirm 1a86:7523 is listed
#        usbipd attach --wsl --hardware-id 1a86:7523  # (one-time: `usbipd bind` as admin)
#   2) In WSL, verify the port appeared:
#        ls /dev/ttyUSB*                              # -> /dev/ttyUSB0
#   3) Build / flash / monitor:
#        source idf-env.sh
#        cd encoder_test          # or BAM/sweep_logger, etc.
#        idf.py -p /dev/ttyUSB0 flash monitor         # Ctrl-] to exit monitor
