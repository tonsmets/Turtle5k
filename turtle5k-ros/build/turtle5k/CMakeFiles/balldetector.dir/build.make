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
CMAKE_SOURCE_DIR = /home/student/turtle5k-git/Turtle5k/turtle5k-ros/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/turtle5k-git/Turtle5k/turtle5k-ros/build

# Include any dependencies generated for this target.
include turtle5k/CMakeFiles/balldetector.dir/depend.make

# Include the progress variables for this target.
include turtle5k/CMakeFiles/balldetector.dir/progress.make

# Include the compile flags for this target's objects.
include turtle5k/CMakeFiles/balldetector.dir/flags.make

turtle5k/CMakeFiles/balldetector.dir/src/balldetector.cpp.o: turtle5k/CMakeFiles/balldetector.dir/flags.make
turtle5k/CMakeFiles/balldetector.dir/src/balldetector.cpp.o: /home/student/turtle5k-git/Turtle5k/turtle5k-ros/src/turtle5k/src/balldetector.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/student/turtle5k-git/Turtle5k/turtle5k-ros/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object turtle5k/CMakeFiles/balldetector.dir/src/balldetector.cpp.o"
	cd /home/student/turtle5k-git/Turtle5k/turtle5k-ros/build/turtle5k && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/balldetector.dir/src/balldetector.cpp.o -c /home/student/turtle5k-git/Turtle5k/turtle5k-ros/src/turtle5k/src/balldetector.cpp

turtle5k/CMakeFiles/balldetector.dir/src/balldetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/balldetector.dir/src/balldetector.cpp.i"
	cd /home/student/turtle5k-git/Turtle5k/turtle5k-ros/build/turtle5k && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/student/turtle5k-git/Turtle5k/turtle5k-ros/src/turtle5k/src/balldetector.cpp > CMakeFiles/balldetector.dir/src/balldetector.cpp.i

turtle5k/CMakeFiles/balldetector.dir/src/balldetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/balldetector.dir/src/balldetector.cpp.s"
	cd /home/student/turtle5k-git/Turtle5k/turtle5k-ros/build/turtle5k && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/student/turtle5k-git/Turtle5k/turtle5k-ros/src/turtle5k/src/balldetector.cpp -o CMakeFiles/balldetector.dir/src/balldetector.cpp.s

turtle5k/CMakeFiles/balldetector.dir/src/balldetector.cpp.o.requires:
.PHONY : turtle5k/CMakeFiles/balldetector.dir/src/balldetector.cpp.o.requires

turtle5k/CMakeFiles/balldetector.dir/src/balldetector.cpp.o.provides: turtle5k/CMakeFiles/balldetector.dir/src/balldetector.cpp.o.requires
	$(MAKE) -f turtle5k/CMakeFiles/balldetector.dir/build.make turtle5k/CMakeFiles/balldetector.dir/src/balldetector.cpp.o.provides.build
.PHONY : turtle5k/CMakeFiles/balldetector.dir/src/balldetector.cpp.o.provides

turtle5k/CMakeFiles/balldetector.dir/src/balldetector.cpp.o.provides.build: turtle5k/CMakeFiles/balldetector.dir/src/balldetector.cpp.o

# Object files for target balldetector
balldetector_OBJECTS = \
"CMakeFiles/balldetector.dir/src/balldetector.cpp.o"

# External object files for target balldetector
balldetector_EXTERNAL_OBJECTS =

/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: turtle5k/CMakeFiles/balldetector.dir/src/balldetector.cpp.o
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: turtle5k/CMakeFiles/balldetector.dir/build.make
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /opt/ros/indigo/lib/libroscpp.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /usr/lib/i386-linux-gnu/libboost_signals.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /usr/lib/i386-linux-gnu/libboost_filesystem.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /opt/ros/indigo/lib/librosconsole.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /usr/lib/liblog4cxx.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /usr/lib/i386-linux-gnu/libboost_regex.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /opt/ros/indigo/lib/librostime.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /usr/lib/i386-linux-gnu/libboost_date_time.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /opt/ros/indigo/lib/libcpp_common.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /usr/lib/i386-linux-gnu/libboost_system.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /usr/lib/i386-linux-gnu/libboost_thread.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /usr/lib/i386-linux-gnu/libpthread.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: /usr/lib/i386-linux-gnu/libconsole_bridge.so
/home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector: turtle5k/CMakeFiles/balldetector.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector"
	cd /home/student/turtle5k-git/Turtle5k/turtle5k-ros/build/turtle5k && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/balldetector.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
turtle5k/CMakeFiles/balldetector.dir/build: /home/student/turtle5k-git/Turtle5k/turtle5k-ros/devel/lib/turtle5k/balldetector
.PHONY : turtle5k/CMakeFiles/balldetector.dir/build

turtle5k/CMakeFiles/balldetector.dir/requires: turtle5k/CMakeFiles/balldetector.dir/src/balldetector.cpp.o.requires
.PHONY : turtle5k/CMakeFiles/balldetector.dir/requires

turtle5k/CMakeFiles/balldetector.dir/clean:
	cd /home/student/turtle5k-git/Turtle5k/turtle5k-ros/build/turtle5k && $(CMAKE_COMMAND) -P CMakeFiles/balldetector.dir/cmake_clean.cmake
.PHONY : turtle5k/CMakeFiles/balldetector.dir/clean

turtle5k/CMakeFiles/balldetector.dir/depend:
	cd /home/student/turtle5k-git/Turtle5k/turtle5k-ros/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/turtle5k-git/Turtle5k/turtle5k-ros/src /home/student/turtle5k-git/Turtle5k/turtle5k-ros/src/turtle5k /home/student/turtle5k-git/Turtle5k/turtle5k-ros/build /home/student/turtle5k-git/Turtle5k/turtle5k-ros/build/turtle5k /home/student/turtle5k-git/Turtle5k/turtle5k-ros/build/turtle5k/CMakeFiles/balldetector.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : turtle5k/CMakeFiles/balldetector.dir/depend

