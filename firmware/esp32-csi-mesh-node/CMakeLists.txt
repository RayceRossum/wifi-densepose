# ESP32-S3 CSI Mesh Node Firmware (Sensor + Mesh Relay)
# Requires ESP-IDF v5.2+
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32-csi-mesh-node)
