# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_SOURCE_DIR = /home/pi/LaneRecog_PI_CntrlIntl_V25

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/LaneRecog_PI_CntrlIntl_V25

# Include any dependencies generated for this target.
include CMakeFiles/LaneRecog.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/LaneRecog.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/LaneRecog.dir/flags.make

CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.o: CMakeFiles/LaneRecog.dir/flags.make
CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.o: TaskScheduling_PI.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/LaneRecog_PI_CntrlIntl_V25/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.o -c /home/pi/LaneRecog_PI_CntrlIntl_V25/TaskScheduling_PI.cpp

CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/LaneRecog_PI_CntrlIntl_V25/TaskScheduling_PI.cpp > CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.i

CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/LaneRecog_PI_CntrlIntl_V25/TaskScheduling_PI.cpp -o CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.s

CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.o.requires:

.PHONY : CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.o.requires

CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.o.provides: CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.o.requires
	$(MAKE) -f CMakeFiles/LaneRecog.dir/build.make CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.o.provides.build
.PHONY : CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.o.provides

CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.o.provides.build: CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.o


CMakeFiles/LaneRecog.dir/lane_recog.cpp.o: CMakeFiles/LaneRecog.dir/flags.make
CMakeFiles/LaneRecog.dir/lane_recog.cpp.o: lane_recog.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/LaneRecog_PI_CntrlIntl_V25/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/LaneRecog.dir/lane_recog.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LaneRecog.dir/lane_recog.cpp.o -c /home/pi/LaneRecog_PI_CntrlIntl_V25/lane_recog.cpp

CMakeFiles/LaneRecog.dir/lane_recog.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LaneRecog.dir/lane_recog.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/LaneRecog_PI_CntrlIntl_V25/lane_recog.cpp > CMakeFiles/LaneRecog.dir/lane_recog.cpp.i

CMakeFiles/LaneRecog.dir/lane_recog.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LaneRecog.dir/lane_recog.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/LaneRecog_PI_CntrlIntl_V25/lane_recog.cpp -o CMakeFiles/LaneRecog.dir/lane_recog.cpp.s

CMakeFiles/LaneRecog.dir/lane_recog.cpp.o.requires:

.PHONY : CMakeFiles/LaneRecog.dir/lane_recog.cpp.o.requires

CMakeFiles/LaneRecog.dir/lane_recog.cpp.o.provides: CMakeFiles/LaneRecog.dir/lane_recog.cpp.o.requires
	$(MAKE) -f CMakeFiles/LaneRecog.dir/build.make CMakeFiles/LaneRecog.dir/lane_recog.cpp.o.provides.build
.PHONY : CMakeFiles/LaneRecog.dir/lane_recog.cpp.o.provides

CMakeFiles/LaneRecog.dir/lane_recog.cpp.o.provides.build: CMakeFiles/LaneRecog.dir/lane_recog.cpp.o


CMakeFiles/LaneRecog.dir/decision_making.cpp.o: CMakeFiles/LaneRecog.dir/flags.make
CMakeFiles/LaneRecog.dir/decision_making.cpp.o: decision_making.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/LaneRecog_PI_CntrlIntl_V25/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/LaneRecog.dir/decision_making.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LaneRecog.dir/decision_making.cpp.o -c /home/pi/LaneRecog_PI_CntrlIntl_V25/decision_making.cpp

CMakeFiles/LaneRecog.dir/decision_making.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LaneRecog.dir/decision_making.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/LaneRecog_PI_CntrlIntl_V25/decision_making.cpp > CMakeFiles/LaneRecog.dir/decision_making.cpp.i

CMakeFiles/LaneRecog.dir/decision_making.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LaneRecog.dir/decision_making.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/LaneRecog_PI_CntrlIntl_V25/decision_making.cpp -o CMakeFiles/LaneRecog.dir/decision_making.cpp.s

CMakeFiles/LaneRecog.dir/decision_making.cpp.o.requires:

.PHONY : CMakeFiles/LaneRecog.dir/decision_making.cpp.o.requires

CMakeFiles/LaneRecog.dir/decision_making.cpp.o.provides: CMakeFiles/LaneRecog.dir/decision_making.cpp.o.requires
	$(MAKE) -f CMakeFiles/LaneRecog.dir/build.make CMakeFiles/LaneRecog.dir/decision_making.cpp.o.provides.build
.PHONY : CMakeFiles/LaneRecog.dir/decision_making.cpp.o.provides

CMakeFiles/LaneRecog.dir/decision_making.cpp.o.provides.build: CMakeFiles/LaneRecog.dir/decision_making.cpp.o


CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.o: CMakeFiles/LaneRecog.dir/flags.make
CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.o: Sign_Reg.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/LaneRecog_PI_CntrlIntl_V25/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.o -c /home/pi/LaneRecog_PI_CntrlIntl_V25/Sign_Reg.cpp

CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/LaneRecog_PI_CntrlIntl_V25/Sign_Reg.cpp > CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.i

CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/LaneRecog_PI_CntrlIntl_V25/Sign_Reg.cpp -o CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.s

CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.o.requires:

.PHONY : CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.o.requires

CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.o.provides: CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.o.requires
	$(MAKE) -f CMakeFiles/LaneRecog.dir/build.make CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.o.provides.build
.PHONY : CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.o.provides

CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.o.provides.build: CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.o


CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.o: CMakeFiles/LaneRecog.dir/flags.make
CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.o: obstacle_detection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/LaneRecog_PI_CntrlIntl_V25/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.o -c /home/pi/LaneRecog_PI_CntrlIntl_V25/obstacle_detection.cpp

CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/LaneRecog_PI_CntrlIntl_V25/obstacle_detection.cpp > CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.i

CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/LaneRecog_PI_CntrlIntl_V25/obstacle_detection.cpp -o CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.s

CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.o.requires:

.PHONY : CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.o.requires

CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.o.provides: CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.o.requires
	$(MAKE) -f CMakeFiles/LaneRecog.dir/build.make CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.o.provides.build
.PHONY : CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.o.provides

CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.o.provides.build: CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.o


# Object files for target LaneRecog
LaneRecog_OBJECTS = \
"CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.o" \
"CMakeFiles/LaneRecog.dir/lane_recog.cpp.o" \
"CMakeFiles/LaneRecog.dir/decision_making.cpp.o" \
"CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.o" \
"CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.o"

# External object files for target LaneRecog
LaneRecog_EXTERNAL_OBJECTS =

LaneRecog: CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.o
LaneRecog: CMakeFiles/LaneRecog.dir/lane_recog.cpp.o
LaneRecog: CMakeFiles/LaneRecog.dir/decision_making.cpp.o
LaneRecog: CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.o
LaneRecog: CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.o
LaneRecog: CMakeFiles/LaneRecog.dir/build.make
LaneRecog: /opt/vc/lib/libmmal_core.so
LaneRecog: /opt/vc/lib/libmmal_util.so
LaneRecog: /opt/vc/lib/libmmal.so
LaneRecog: /opt/vc/lib/libmmal_core.so
LaneRecog: /opt/vc/lib/libmmal_util.so
LaneRecog: /opt/vc/lib/libmmal.so
LaneRecog: /usr/local/lib/libopencv_shape.so.3.2.0
LaneRecog: /usr/local/lib/libopencv_stitching.so.3.2.0
LaneRecog: /usr/local/lib/libopencv_superres.so.3.2.0
LaneRecog: /usr/local/lib/libopencv_videostab.so.3.2.0
LaneRecog: /usr/local/lib/libopencv_objdetect.so.3.2.0
LaneRecog: /usr/local/lib/libopencv_calib3d.so.3.2.0
LaneRecog: /usr/local/lib/libopencv_features2d.so.3.2.0
LaneRecog: /usr/local/lib/libopencv_flann.so.3.2.0
LaneRecog: /usr/local/lib/libopencv_highgui.so.3.2.0
LaneRecog: /usr/local/lib/libopencv_ml.so.3.2.0
LaneRecog: /usr/local/lib/libopencv_photo.so.3.2.0
LaneRecog: /usr/local/lib/libopencv_video.so.3.2.0
LaneRecog: /usr/local/lib/libopencv_videoio.so.3.2.0
LaneRecog: /usr/local/lib/libopencv_imgcodecs.so.3.2.0
LaneRecog: /usr/local/lib/libopencv_imgproc.so.3.2.0
LaneRecog: /usr/local/lib/libopencv_core.so.3.2.0
LaneRecog: CMakeFiles/LaneRecog.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/LaneRecog_PI_CntrlIntl_V25/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable LaneRecog"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LaneRecog.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/LaneRecog.dir/build: LaneRecog

.PHONY : CMakeFiles/LaneRecog.dir/build

CMakeFiles/LaneRecog.dir/requires: CMakeFiles/LaneRecog.dir/TaskScheduling_PI.cpp.o.requires
CMakeFiles/LaneRecog.dir/requires: CMakeFiles/LaneRecog.dir/lane_recog.cpp.o.requires
CMakeFiles/LaneRecog.dir/requires: CMakeFiles/LaneRecog.dir/decision_making.cpp.o.requires
CMakeFiles/LaneRecog.dir/requires: CMakeFiles/LaneRecog.dir/Sign_Reg.cpp.o.requires
CMakeFiles/LaneRecog.dir/requires: CMakeFiles/LaneRecog.dir/obstacle_detection.cpp.o.requires

.PHONY : CMakeFiles/LaneRecog.dir/requires

CMakeFiles/LaneRecog.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/LaneRecog.dir/cmake_clean.cmake
.PHONY : CMakeFiles/LaneRecog.dir/clean

CMakeFiles/LaneRecog.dir/depend:
	cd /home/pi/LaneRecog_PI_CntrlIntl_V25 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/LaneRecog_PI_CntrlIntl_V25 /home/pi/LaneRecog_PI_CntrlIntl_V25 /home/pi/LaneRecog_PI_CntrlIntl_V25 /home/pi/LaneRecog_PI_CntrlIntl_V25 /home/pi/LaneRecog_PI_CntrlIntl_V25/CMakeFiles/LaneRecog.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/LaneRecog.dir/depend

