# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/pico/pico-sdk/tools/pioasm"
  "C:/pico/PICO_Programs/build/pioasm"
  "C:/pico/PICO_Programs/build/pioasm"
  "C:/pico/PICO_Programs/build/pioasm/tmp"
  "C:/pico/PICO_Programs/build/pioasm/src/PioasmBuild-stamp"
  "C:/pico/PICO_Programs/build/pioasm/src"
  "C:/pico/PICO_Programs/build/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/pico/PICO_Programs/build/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
