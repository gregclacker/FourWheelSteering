# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Chapa/esp/v4.4.6/esp-idf/components/bootloader/subproject"
  "C:/Users/Chapa/OneDrive/Desktop/UTA/2025/Spring 25/MAE 4287 SENIOR DESIGN 1/Code/ESP32_FWS_Build/FourWheelSteering/build/bootloader"
  "C:/Users/Chapa/OneDrive/Desktop/UTA/2025/Spring 25/MAE 4287 SENIOR DESIGN 1/Code/ESP32_FWS_Build/FourWheelSteering/build/bootloader-prefix"
  "C:/Users/Chapa/OneDrive/Desktop/UTA/2025/Spring 25/MAE 4287 SENIOR DESIGN 1/Code/ESP32_FWS_Build/FourWheelSteering/build/bootloader-prefix/tmp"
  "C:/Users/Chapa/OneDrive/Desktop/UTA/2025/Spring 25/MAE 4287 SENIOR DESIGN 1/Code/ESP32_FWS_Build/FourWheelSteering/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/Chapa/OneDrive/Desktop/UTA/2025/Spring 25/MAE 4287 SENIOR DESIGN 1/Code/ESP32_FWS_Build/FourWheelSteering/build/bootloader-prefix/src"
  "C:/Users/Chapa/OneDrive/Desktop/UTA/2025/Spring 25/MAE 4287 SENIOR DESIGN 1/Code/ESP32_FWS_Build/FourWheelSteering/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/Chapa/OneDrive/Desktop/UTA/2025/Spring 25/MAE 4287 SENIOR DESIGN 1/Code/ESP32_FWS_Build/FourWheelSteering/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
