﻿# CMAKE generated file: DO NOT EDIT!
# Generated by "NMake Makefiles" Generator, CMake Version 3.23

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

!IF "$(OS)" == "Windows_NT"
NULL=
!ELSE
NULL=nul
!ENDIF
SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\pico\PICO_Programs\myBlink

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\pico\PICO_Programs\build

# Utility rule file for ELF2UF2Build.

# Include any custom commands dependencies for this target.
include CMakeFiles\ELF2UF2Build.dir\compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles\ELF2UF2Build.dir\progress.make

CMakeFiles\ELF2UF2Build: CMakeFiles\ELF2UF2Build-complete

CMakeFiles\ELF2UF2Build-complete: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-install
CMakeFiles\ELF2UF2Build-complete: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-mkdir
CMakeFiles\ELF2UF2Build-complete: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-download
CMakeFiles\ELF2UF2Build-complete: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-update
CMakeFiles\ELF2UF2Build-complete: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-patch
CMakeFiles\ELF2UF2Build-complete: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-configure
CMakeFiles\ELF2UF2Build-complete: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-build
CMakeFiles\ELF2UF2Build-complete: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\pico\PICO_Programs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'ELF2UF2Build'"
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E make_directory C:/pico/PICO_Programs/build/CMakeFiles
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/pico/PICO_Programs/build/CMakeFiles/ELF2UF2Build-complete
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/pico/PICO_Programs/build/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-done

elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-build: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\pico\PICO_Programs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing build step for 'ELF2UF2Build'"
	cd C:\pico\PICO_Programs\build\elf2uf2
	$(MAKE)
	cd C:\pico\PICO_Programs\build

elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-configure: elf2uf2\tmp\ELF2UF2Build-cfgcmd.txt
elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-configure: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\pico\PICO_Programs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Performing configure step for 'ELF2UF2Build'"
	cd C:\pico\PICO_Programs\build\elf2uf2
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" "-GNMake Makefiles" C:/pico/pico-sdk/tools/elf2uf2
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/pico/PICO_Programs/build/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-configure
	cd C:\pico\PICO_Programs\build

elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-download: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-source_dirinfo.txt
elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-download: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\pico\PICO_Programs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "No download step for 'ELF2UF2Build'"
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/pico/PICO_Programs/build/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-download

elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-install: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\pico\PICO_Programs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No install step for 'ELF2UF2Build'"
	cd C:\pico\PICO_Programs\build\elf2uf2
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/pico/PICO_Programs/build/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-install
	cd C:\pico\PICO_Programs\build

elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\pico\PICO_Programs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Creating directories for 'ELF2UF2Build'"
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -P C:/pico/PICO_Programs/build/elf2uf2/tmp/ELF2UF2Build-mkdirs.cmake
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/pico/PICO_Programs/build/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-mkdir

elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-patch: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-update
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\pico\PICO_Programs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No patch step for 'ELF2UF2Build'"
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/pico/PICO_Programs/build/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-patch

elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-update: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\pico\PICO_Programs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "No update step for 'ELF2UF2Build'"
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/pico/PICO_Programs/build/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-update

ELF2UF2Build: CMakeFiles\ELF2UF2Build
ELF2UF2Build: CMakeFiles\ELF2UF2Build-complete
ELF2UF2Build: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-build
ELF2UF2Build: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-configure
ELF2UF2Build: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-download
ELF2UF2Build: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-install
ELF2UF2Build: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-mkdir
ELF2UF2Build: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-patch
ELF2UF2Build: elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-update
ELF2UF2Build: CMakeFiles\ELF2UF2Build.dir\build.make
.PHONY : ELF2UF2Build

# Rule to build all files generated by this target.
CMakeFiles\ELF2UF2Build.dir\build: ELF2UF2Build
.PHONY : CMakeFiles\ELF2UF2Build.dir\build

CMakeFiles\ELF2UF2Build.dir\clean:
	$(CMAKE_COMMAND) -P CMakeFiles\ELF2UF2Build.dir\cmake_clean.cmake
.PHONY : CMakeFiles\ELF2UF2Build.dir\clean

CMakeFiles\ELF2UF2Build.dir\depend:
	$(CMAKE_COMMAND) -E cmake_depends "NMake Makefiles" C:\pico\PICO_Programs\myBlink C:\pico\PICO_Programs\myBlink C:\pico\PICO_Programs\build C:\pico\PICO_Programs\build C:\pico\PICO_Programs\build\CMakeFiles\ELF2UF2Build.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles\ELF2UF2Build.dir\depend

