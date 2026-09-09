# Standalone WS2812 wiring/solder test for the ESP32-C6 SuperMini.
# Kept out of the Matter app on purpose: it builds and flashes in seconds.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(led_test)
