cmake_minimum_required(VERSION 3.16)

# Reuse the same vendored SH-2 component as the real controller.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../pid/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(imu_noise)
