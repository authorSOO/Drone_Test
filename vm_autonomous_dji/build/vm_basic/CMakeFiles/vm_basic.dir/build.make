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
CMAKE_SOURCE_DIR = /home/ethan/hector_drone/src/vm_autonomous_dji/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ethan/hector_drone/src/vm_autonomous_dji/build

# Include any dependencies generated for this target.
include vm_basic/CMakeFiles/vm_basic.dir/depend.make

# Include the progress variables for this target.
include vm_basic/CMakeFiles/vm_basic.dir/progress.make

# Include the compile flags for this target's objects.
include vm_basic/CMakeFiles/vm_basic.dir/flags.make

vm_basic/CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.o: vm_basic/CMakeFiles/vm_basic.dir/flags.make
vm_basic/CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.o: /home/ethan/hector_drone/src/vm_autonomous_dji/src/vm_basic/src/vm_basic_command.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ethan/hector_drone/src/vm_autonomous_dji/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object vm_basic/CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.o"
	cd /home/ethan/hector_drone/src/vm_autonomous_dji/build/vm_basic && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.o -c /home/ethan/hector_drone/src/vm_autonomous_dji/src/vm_basic/src/vm_basic_command.cpp

vm_basic/CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.i"
	cd /home/ethan/hector_drone/src/vm_autonomous_dji/build/vm_basic && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ethan/hector_drone/src/vm_autonomous_dji/src/vm_basic/src/vm_basic_command.cpp > CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.i

vm_basic/CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.s"
	cd /home/ethan/hector_drone/src/vm_autonomous_dji/build/vm_basic && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ethan/hector_drone/src/vm_autonomous_dji/src/vm_basic/src/vm_basic_command.cpp -o CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.s

vm_basic/CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.o.requires:

.PHONY : vm_basic/CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.o.requires

vm_basic/CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.o.provides: vm_basic/CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.o.requires
	$(MAKE) -f vm_basic/CMakeFiles/vm_basic.dir/build.make vm_basic/CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.o.provides.build
.PHONY : vm_basic/CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.o.provides

vm_basic/CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.o.provides.build: vm_basic/CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.o


vm_basic/CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.o: vm_basic/CMakeFiles/vm_basic.dir/flags.make
vm_basic/CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.o: /home/ethan/hector_drone/src/vm_autonomous_dji/src/vm_basic/src/vm_command_keyboard.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ethan/hector_drone/src/vm_autonomous_dji/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object vm_basic/CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.o"
	cd /home/ethan/hector_drone/src/vm_autonomous_dji/build/vm_basic && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.o -c /home/ethan/hector_drone/src/vm_autonomous_dji/src/vm_basic/src/vm_command_keyboard.cpp

vm_basic/CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.i"
	cd /home/ethan/hector_drone/src/vm_autonomous_dji/build/vm_basic && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ethan/hector_drone/src/vm_autonomous_dji/src/vm_basic/src/vm_command_keyboard.cpp > CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.i

vm_basic/CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.s"
	cd /home/ethan/hector_drone/src/vm_autonomous_dji/build/vm_basic && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ethan/hector_drone/src/vm_autonomous_dji/src/vm_basic/src/vm_command_keyboard.cpp -o CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.s

vm_basic/CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.o.requires:

.PHONY : vm_basic/CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.o.requires

vm_basic/CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.o.provides: vm_basic/CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.o.requires
	$(MAKE) -f vm_basic/CMakeFiles/vm_basic.dir/build.make vm_basic/CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.o.provides.build
.PHONY : vm_basic/CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.o.provides

vm_basic/CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.o.provides.build: vm_basic/CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.o


# Object files for target vm_basic
vm_basic_OBJECTS = \
"CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.o" \
"CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.o"

# External object files for target vm_basic
vm_basic_EXTERNAL_OBJECTS =

/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: vm_basic/CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.o
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: vm_basic/CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.o
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: vm_basic/CMakeFiles/vm_basic.dir/build.make
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /home/ethan/hector_drone/devel/lib/libhector_quadrotor_interface.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/libclass_loader.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/libPocoFoundation.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/libdl.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/libroslib.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/librospack.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/liborocos-kdl.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/liborocos-kdl.so.1.3.2
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/libtf2_ros.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/libactionlib.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/libmessage_filters.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/libroscpp.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/librosconsole.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/libtf2.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/librostime.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /opt/ros/kinetic/lib/libcpp_common.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic: vm_basic/CMakeFiles/vm_basic.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ethan/hector_drone/src/vm_autonomous_dji/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic"
	cd /home/ethan/hector_drone/src/vm_autonomous_dji/build/vm_basic && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vm_basic.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vm_basic/CMakeFiles/vm_basic.dir/build: /home/ethan/hector_drone/src/vm_autonomous_dji/devel/lib/vm_basic/vm_basic

.PHONY : vm_basic/CMakeFiles/vm_basic.dir/build

vm_basic/CMakeFiles/vm_basic.dir/requires: vm_basic/CMakeFiles/vm_basic.dir/src/vm_basic_command.cpp.o.requires
vm_basic/CMakeFiles/vm_basic.dir/requires: vm_basic/CMakeFiles/vm_basic.dir/src/vm_command_keyboard.cpp.o.requires

.PHONY : vm_basic/CMakeFiles/vm_basic.dir/requires

vm_basic/CMakeFiles/vm_basic.dir/clean:
	cd /home/ethan/hector_drone/src/vm_autonomous_dji/build/vm_basic && $(CMAKE_COMMAND) -P CMakeFiles/vm_basic.dir/cmake_clean.cmake
.PHONY : vm_basic/CMakeFiles/vm_basic.dir/clean

vm_basic/CMakeFiles/vm_basic.dir/depend:
	cd /home/ethan/hector_drone/src/vm_autonomous_dji/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ethan/hector_drone/src/vm_autonomous_dji/src /home/ethan/hector_drone/src/vm_autonomous_dji/src/vm_basic /home/ethan/hector_drone/src/vm_autonomous_dji/build /home/ethan/hector_drone/src/vm_autonomous_dji/build/vm_basic /home/ethan/hector_drone/src/vm_autonomous_dji/build/vm_basic/CMakeFiles/vm_basic.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vm_basic/CMakeFiles/vm_basic.dir/depend

