# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/apoorva/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/apoorva/catkin_ws/build

# Include any dependencies generated for this target.
include vicon_bridge/CMakeFiles/calibrate.dir/depend.make

# Include the progress variables for this target.
include vicon_bridge/CMakeFiles/calibrate.dir/progress.make

# Include the compile flags for this target's objects.
include vicon_bridge/CMakeFiles/calibrate.dir/flags.make

vicon_bridge/CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.o: vicon_bridge/CMakeFiles/calibrate.dir/flags.make
vicon_bridge/CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.o: /home/apoorva/catkin_ws/src/vicon_bridge/src/calibrate_segment.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/apoorva/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object vicon_bridge/CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.o"
	cd /home/apoorva/catkin_ws/build/vicon_bridge && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.o -c /home/apoorva/catkin_ws/src/vicon_bridge/src/calibrate_segment.cpp

vicon_bridge/CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.i"
	cd /home/apoorva/catkin_ws/build/vicon_bridge && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/apoorva/catkin_ws/src/vicon_bridge/src/calibrate_segment.cpp > CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.i

vicon_bridge/CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.s"
	cd /home/apoorva/catkin_ws/build/vicon_bridge && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/apoorva/catkin_ws/src/vicon_bridge/src/calibrate_segment.cpp -o CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.s

vicon_bridge/CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.o.requires:
.PHONY : vicon_bridge/CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.o.requires

vicon_bridge/CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.o.provides: vicon_bridge/CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.o.requires
	$(MAKE) -f vicon_bridge/CMakeFiles/calibrate.dir/build.make vicon_bridge/CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.o.provides.build
.PHONY : vicon_bridge/CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.o.provides

vicon_bridge/CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.o.provides.build: vicon_bridge/CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.o

# Object files for target calibrate
calibrate_OBJECTS = \
"CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.o"

# External object files for target calibrate
calibrate_EXTERNAL_OBJECTS =

/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: vicon_bridge/CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.o
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: vicon_bridge/CMakeFiles/calibrate.dir/build.make
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /opt/ros/indigo/lib/libtf.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /opt/ros/indigo/lib/libtf2_ros.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /opt/ros/indigo/lib/libactionlib.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /opt/ros/indigo/lib/libmessage_filters.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /opt/ros/indigo/lib/libtf2.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /opt/ros/indigo/lib/libroscpp.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /opt/ros/indigo/lib/librosconsole.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /usr/lib/liblog4cxx.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /opt/ros/indigo/lib/librostime.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /opt/ros/indigo/lib/libcpp_common.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate: vicon_bridge/CMakeFiles/calibrate.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate"
	cd /home/apoorva/catkin_ws/build/vicon_bridge && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/calibrate.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vicon_bridge/CMakeFiles/calibrate.dir/build: /home/apoorva/catkin_ws/devel/lib/vicon_bridge/calibrate
.PHONY : vicon_bridge/CMakeFiles/calibrate.dir/build

vicon_bridge/CMakeFiles/calibrate.dir/requires: vicon_bridge/CMakeFiles/calibrate.dir/src/calibrate_segment.cpp.o.requires
.PHONY : vicon_bridge/CMakeFiles/calibrate.dir/requires

vicon_bridge/CMakeFiles/calibrate.dir/clean:
	cd /home/apoorva/catkin_ws/build/vicon_bridge && $(CMAKE_COMMAND) -P CMakeFiles/calibrate.dir/cmake_clean.cmake
.PHONY : vicon_bridge/CMakeFiles/calibrate.dir/clean

vicon_bridge/CMakeFiles/calibrate.dir/depend:
	cd /home/apoorva/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/apoorva/catkin_ws/src /home/apoorva/catkin_ws/src/vicon_bridge /home/apoorva/catkin_ws/build /home/apoorva/catkin_ws/build/vicon_bridge /home/apoorva/catkin_ws/build/vicon_bridge/CMakeFiles/calibrate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vicon_bridge/CMakeFiles/calibrate.dir/depend

