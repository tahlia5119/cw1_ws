# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/tahlia/cw1/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tahlia/cw1/build

# Include any dependencies generated for this target.
include comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/depend.make

# Include the progress variables for this target.
include comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/progress.make

# Include the compile flags for this target's objects.
include comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/flags.make

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.o: comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/flags.make
comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.o: /home/tahlia/cw1/src/comp313p/stdr_simulator/stdr_server/src/stdr_server_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tahlia/cw1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.o"
	cd /home/tahlia/cw1/build/comp313p/stdr_simulator/stdr_server && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.o -c /home/tahlia/cw1/src/comp313p/stdr_simulator/stdr_server/src/stdr_server_node.cpp

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.i"
	cd /home/tahlia/cw1/build/comp313p/stdr_simulator/stdr_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tahlia/cw1/src/comp313p/stdr_simulator/stdr_server/src/stdr_server_node.cpp > CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.i

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.s"
	cd /home/tahlia/cw1/build/comp313p/stdr_simulator/stdr_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tahlia/cw1/src/comp313p/stdr_simulator/stdr_server/src/stdr_server_node.cpp -o CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.s

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.o.requires:

.PHONY : comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.o.requires

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.o.provides: comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.o.requires
	$(MAKE) -f comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/build.make comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.o.provides.build
.PHONY : comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.o.provides

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.o.provides.build: comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.o


# Object files for target stdr_server_node
stdr_server_node_OBJECTS = \
"CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.o"

# External object files for target stdr_server_node
stdr_server_node_EXTERNAL_OBJECTS =

/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.o
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/build.make
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /home/tahlia/cw1/devel/lib/libstdr_server.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libtf.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libtf2_ros.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libmessage_filters.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libactionlib.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libnodeletlib.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libbondcpp.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libclass_loader.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/libPocoFoundation.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libdl.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libroslib.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/librospack.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libmap_server_image_loader.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libroscpp.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/librosconsole.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libtf2.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/librostime.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libcpp_common.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /home/tahlia/cw1/devel/lib/libstdr_map_server.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /home/tahlia/cw1/devel/lib/libstdr_map_loader.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libtf.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libtf2_ros.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libmessage_filters.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libactionlib.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libnodeletlib.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libbondcpp.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libclass_loader.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/libPocoFoundation.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libdl.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libroslib.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/librospack.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libmap_server_image_loader.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libroscpp.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/librosconsole.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libtf2.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/librostime.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /opt/ros/kinetic/lib/libcpp_common.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node: comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tahlia/cw1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node"
	cd /home/tahlia/cw1/build/comp313p/stdr_simulator/stdr_server && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stdr_server_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/build: /home/tahlia/cw1/devel/lib/stdr_server/stdr_server_node

.PHONY : comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/build

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/requires: comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/src/stdr_server_node.cpp.o.requires

.PHONY : comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/requires

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/clean:
	cd /home/tahlia/cw1/build/comp313p/stdr_simulator/stdr_server && $(CMAKE_COMMAND) -P CMakeFiles/stdr_server_node.dir/cmake_clean.cmake
.PHONY : comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/clean

comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/depend:
	cd /home/tahlia/cw1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tahlia/cw1/src /home/tahlia/cw1/src/comp313p/stdr_simulator/stdr_server /home/tahlia/cw1/build /home/tahlia/cw1/build/comp313p/stdr_simulator/stdr_server /home/tahlia/cw1/build/comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : comp313p/stdr_simulator/stdr_server/CMakeFiles/stdr_server_node.dir/depend

