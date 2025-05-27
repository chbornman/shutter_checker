#!/bin/bash

# Source ESP-IDF environment
if [ -f "$HOME/esp/esp-idf/export.sh" ]; then
    . $HOME/esp/esp-idf/export.sh
elif [ -f "$HOME/esp-idf/export.sh" ]; then
    . $HOME/esp-idf/export.sh
else
    echo "ESP-IDF not found. Please install ESP-IDF first."
    exit 1
fi

# Build the project
idf.py build