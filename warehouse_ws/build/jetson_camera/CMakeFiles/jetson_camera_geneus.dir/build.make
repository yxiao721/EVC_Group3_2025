# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

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

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/jetbot/.local/lib/python3.6/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/jetbot/.local/lib/python3.6/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jetbot/EVC/workshops/FINALPROJECT/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jetbot/EVC/workshops/FINALPROJECT/build

# Utility rule file for jetson_camera_geneus.

# Include any custom commands dependencies for this target.
include jetson_camera/CMakeFiles/jetson_camera_geneus.dir/compiler_depend.make

# Include the progress variables for this target.
include jetson_camera/CMakeFiles/jetson_camera_geneus.dir/progress.make

jetson_camera_geneus: jetson_camera/CMakeFiles/jetson_camera_geneus.dir/build.make
.PHONY : jetson_camera_geneus

# Rule to build all files generated by this target.
jetson_camera/CMakeFiles/jetson_camera_geneus.dir/build: jetson_camera_geneus
.PHONY : jetson_camera/CMakeFiles/jetson_camera_geneus.dir/build

jetson_camera/CMakeFiles/jetson_camera_geneus.dir/clean:
	cd /home/jetbot/EVC/workshops/FINALPROJECT/build/jetson_camera && $(CMAKE_COMMAND) -P CMakeFiles/jetson_camera_geneus.dir/cmake_clean.cmake
.PHONY : jetson_camera/CMakeFiles/jetson_camera_geneus.dir/clean

jetson_camera/CMakeFiles/jetson_camera_geneus.dir/depend:
	cd /home/jetbot/EVC/workshops/FINALPROJECT/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jetbot/EVC/workshops/FINALPROJECT/src /home/jetbot/EVC/workshops/FINALPROJECT/src/jetson_camera /home/jetbot/EVC/workshops/FINALPROJECT/build /home/jetbot/EVC/workshops/FINALPROJECT/build/jetson_camera /home/jetbot/EVC/workshops/FINALPROJECT/build/jetson_camera/CMakeFiles/jetson_camera_geneus.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : jetson_camera/CMakeFiles/jetson_camera_geneus.dir/depend

