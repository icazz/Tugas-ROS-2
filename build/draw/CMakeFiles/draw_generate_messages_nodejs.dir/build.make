# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zika/drawing/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zika/drawing/build

# Utility rule file for draw_generate_messages_nodejs.

# Include the progress variables for this target.
include draw/CMakeFiles/draw_generate_messages_nodejs.dir/progress.make

draw_generate_messages_nodejs: draw/CMakeFiles/draw_generate_messages_nodejs.dir/build.make

.PHONY : draw_generate_messages_nodejs

# Rule to build all files generated by this target.
draw/CMakeFiles/draw_generate_messages_nodejs.dir/build: draw_generate_messages_nodejs

.PHONY : draw/CMakeFiles/draw_generate_messages_nodejs.dir/build

draw/CMakeFiles/draw_generate_messages_nodejs.dir/clean:
	cd /home/zika/drawing/build/draw && $(CMAKE_COMMAND) -P CMakeFiles/draw_generate_messages_nodejs.dir/cmake_clean.cmake
.PHONY : draw/CMakeFiles/draw_generate_messages_nodejs.dir/clean

draw/CMakeFiles/draw_generate_messages_nodejs.dir/depend:
	cd /home/zika/drawing/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zika/drawing/src /home/zika/drawing/src/draw /home/zika/drawing/build /home/zika/drawing/build/draw /home/zika/drawing/build/draw/CMakeFiles/draw_generate_messages_nodejs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : draw/CMakeFiles/draw_generate_messages_nodejs.dir/depend

