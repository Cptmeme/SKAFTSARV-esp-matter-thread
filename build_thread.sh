#!/bin/sh
# Build the SKAFTSÄRV Matter-over-Thread firmware.
#
# The stock `source $IDF_PATH/export.sh` does not work on this machine: it
# picks up Homebrew python 3.14 and then looks for a venv that does not exist.
# So the tool environment is pulled straight out of idf_tools.py instead.
set -e

# Override any of these in your environment if your install differs.
: "${IDF_PATH:=$HOME/.espressif/v5.5.4/esp-idf}"
: "${IDF_TOOLS_PATH:=$HOME/.espressif}"
: "${ESP_MATTER_PATH:=$HOME/.espressif/esp-matter}"
export IDF_PATH IDF_TOOLS_PATH ESP_MATTER_PATH
# This install has no espidf.constraints.v5.5.txt (the VS Code extension
# manages its own python env), and idf.py refuses to run any action
# without it. The packages are present, so skip the check.
export IDF_PYTHON_CHECK_CONSTRAINTS=no
: "${IDF_PYTHON_ENV_PATH:=$HOME/.espressif/tools/python/v5.5.4/venv}"
export IDF_PYTHON_ENV_PATH
IDF_PY="$IDF_PYTHON_ENV_PATH/bin/python"

eval "$("$IDF_PY" "$IDF_PATH/tools/idf_tools.py" export --format key-value 2>/dev/null \
        | grep -E '^[A-Za-z_]+=' | sed 's/^/export /')"

# What esp-matter's own export.sh adds: gn (the CHIP component builds with it),
# the host tools, and zap.
CHIP=$ESP_MATTER_PATH/connectedhomeip/connectedhomeip
export PATH="$PATH:$CHIP/.environment/cipd/packages/pigweed/:$CHIP/out/host"
export ZAP_INSTALL_PATH="$CHIP/.environment/cipd/packages/zap"
export _PW_ACTUAL_ENVIRONMENT_ROOT="$CHIP/.environment"

cd "$(cd "$(dirname "$0")" && pwd)"
exec "$IDF_PY" "$IDF_PATH/tools/idf.py" "$@"
