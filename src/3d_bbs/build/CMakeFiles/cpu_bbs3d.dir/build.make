# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jgy/RMUC_simulation/src/3d_bbs

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jgy/RMUC_simulation/src/3d_bbs/build

# Include any dependencies generated for this target.
include CMakeFiles/cpu_bbs3d.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/cpu_bbs3d.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/cpu_bbs3d.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cpu_bbs3d.dir/flags.make

CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/bbs3d.cpp.o: CMakeFiles/cpu_bbs3d.dir/flags.make
CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/bbs3d.cpp.o: /home/jgy/RMUC_simulation/src/3d_bbs/bbs3d/src/cpu_bbs3d/bbs3d.cpp
CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/bbs3d.cpp.o: CMakeFiles/cpu_bbs3d.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jgy/RMUC_simulation/src/3d_bbs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/bbs3d.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/bbs3d.cpp.o -MF CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/bbs3d.cpp.o.d -o CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/bbs3d.cpp.o -c /home/jgy/RMUC_simulation/src/3d_bbs/bbs3d/src/cpu_bbs3d/bbs3d.cpp

CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/bbs3d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/bbs3d.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jgy/RMUC_simulation/src/3d_bbs/bbs3d/src/cpu_bbs3d/bbs3d.cpp > CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/bbs3d.cpp.i

CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/bbs3d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/bbs3d.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jgy/RMUC_simulation/src/3d_bbs/bbs3d/src/cpu_bbs3d/bbs3d.cpp -o CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/bbs3d.cpp.s

CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps.cpp.o: CMakeFiles/cpu_bbs3d.dir/flags.make
CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps.cpp.o: /home/jgy/RMUC_simulation/src/3d_bbs/bbs3d/src/cpu_bbs3d/voxelmaps.cpp
CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps.cpp.o: CMakeFiles/cpu_bbs3d.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jgy/RMUC_simulation/src/3d_bbs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps.cpp.o -MF CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps.cpp.o.d -o CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps.cpp.o -c /home/jgy/RMUC_simulation/src/3d_bbs/bbs3d/src/cpu_bbs3d/voxelmaps.cpp

CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jgy/RMUC_simulation/src/3d_bbs/bbs3d/src/cpu_bbs3d/voxelmaps.cpp > CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps.cpp.i

CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jgy/RMUC_simulation/src/3d_bbs/bbs3d/src/cpu_bbs3d/voxelmaps.cpp -o CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps.cpp.s

CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp.o: CMakeFiles/cpu_bbs3d.dir/flags.make
CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp.o: /home/jgy/RMUC_simulation/src/3d_bbs/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp
CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp.o: CMakeFiles/cpu_bbs3d.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jgy/RMUC_simulation/src/3d_bbs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp.o -MF CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp.o.d -o CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp.o -c /home/jgy/RMUC_simulation/src/3d_bbs/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp

CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jgy/RMUC_simulation/src/3d_bbs/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp > CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp.i

CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jgy/RMUC_simulation/src/3d_bbs/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp -o CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp.s

# Object files for target cpu_bbs3d
cpu_bbs3d_OBJECTS = \
"CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/bbs3d.cpp.o" \
"CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps.cpp.o" \
"CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp.o"

# External object files for target cpu_bbs3d
cpu_bbs3d_EXTERNAL_OBJECTS =

libcpu_bbs3d.so: CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/bbs3d.cpp.o
libcpu_bbs3d.so: CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps.cpp.o
libcpu_bbs3d.so: CMakeFiles/cpu_bbs3d.dir/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp.o
libcpu_bbs3d.so: CMakeFiles/cpu_bbs3d.dir/build.make
libcpu_bbs3d.so: CMakeFiles/cpu_bbs3d.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jgy/RMUC_simulation/src/3d_bbs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library libcpu_bbs3d.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cpu_bbs3d.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cpu_bbs3d.dir/build: libcpu_bbs3d.so
.PHONY : CMakeFiles/cpu_bbs3d.dir/build

CMakeFiles/cpu_bbs3d.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cpu_bbs3d.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cpu_bbs3d.dir/clean

CMakeFiles/cpu_bbs3d.dir/depend:
	cd /home/jgy/RMUC_simulation/src/3d_bbs/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jgy/RMUC_simulation/src/3d_bbs /home/jgy/RMUC_simulation/src/3d_bbs /home/jgy/RMUC_simulation/src/3d_bbs/build /home/jgy/RMUC_simulation/src/3d_bbs/build /home/jgy/RMUC_simulation/src/3d_bbs/build/CMakeFiles/cpu_bbs3d.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cpu_bbs3d.dir/depend

