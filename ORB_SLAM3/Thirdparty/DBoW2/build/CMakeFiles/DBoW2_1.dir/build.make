# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/build

# Include any dependencies generated for this target.
include CMakeFiles/DBoW2_1.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/DBoW2_1.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/DBoW2_1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/DBoW2_1.dir/flags.make

CMakeFiles/DBoW2_1.dir/DBoW2/BowVector.cpp.o: CMakeFiles/DBoW2_1.dir/flags.make
CMakeFiles/DBoW2_1.dir/DBoW2/BowVector.cpp.o: ../DBoW2/BowVector.cpp
CMakeFiles/DBoW2_1.dir/DBoW2/BowVector.cpp.o: CMakeFiles/DBoW2_1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/DBoW2_1.dir/DBoW2/BowVector.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/DBoW2_1.dir/DBoW2/BowVector.cpp.o -MF CMakeFiles/DBoW2_1.dir/DBoW2/BowVector.cpp.o.d -o CMakeFiles/DBoW2_1.dir/DBoW2/BowVector.cpp.o -c /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DBoW2/BowVector.cpp

CMakeFiles/DBoW2_1.dir/DBoW2/BowVector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DBoW2_1.dir/DBoW2/BowVector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DBoW2/BowVector.cpp > CMakeFiles/DBoW2_1.dir/DBoW2/BowVector.cpp.i

CMakeFiles/DBoW2_1.dir/DBoW2/BowVector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DBoW2_1.dir/DBoW2/BowVector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DBoW2/BowVector.cpp -o CMakeFiles/DBoW2_1.dir/DBoW2/BowVector.cpp.s

CMakeFiles/DBoW2_1.dir/DBoW2/FORB.cpp.o: CMakeFiles/DBoW2_1.dir/flags.make
CMakeFiles/DBoW2_1.dir/DBoW2/FORB.cpp.o: ../DBoW2/FORB.cpp
CMakeFiles/DBoW2_1.dir/DBoW2/FORB.cpp.o: CMakeFiles/DBoW2_1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/DBoW2_1.dir/DBoW2/FORB.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/DBoW2_1.dir/DBoW2/FORB.cpp.o -MF CMakeFiles/DBoW2_1.dir/DBoW2/FORB.cpp.o.d -o CMakeFiles/DBoW2_1.dir/DBoW2/FORB.cpp.o -c /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DBoW2/FORB.cpp

CMakeFiles/DBoW2_1.dir/DBoW2/FORB.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DBoW2_1.dir/DBoW2/FORB.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DBoW2/FORB.cpp > CMakeFiles/DBoW2_1.dir/DBoW2/FORB.cpp.i

CMakeFiles/DBoW2_1.dir/DBoW2/FORB.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DBoW2_1.dir/DBoW2/FORB.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DBoW2/FORB.cpp -o CMakeFiles/DBoW2_1.dir/DBoW2/FORB.cpp.s

CMakeFiles/DBoW2_1.dir/DBoW2/FeatureVector.cpp.o: CMakeFiles/DBoW2_1.dir/flags.make
CMakeFiles/DBoW2_1.dir/DBoW2/FeatureVector.cpp.o: ../DBoW2/FeatureVector.cpp
CMakeFiles/DBoW2_1.dir/DBoW2/FeatureVector.cpp.o: CMakeFiles/DBoW2_1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/DBoW2_1.dir/DBoW2/FeatureVector.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/DBoW2_1.dir/DBoW2/FeatureVector.cpp.o -MF CMakeFiles/DBoW2_1.dir/DBoW2/FeatureVector.cpp.o.d -o CMakeFiles/DBoW2_1.dir/DBoW2/FeatureVector.cpp.o -c /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DBoW2/FeatureVector.cpp

CMakeFiles/DBoW2_1.dir/DBoW2/FeatureVector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DBoW2_1.dir/DBoW2/FeatureVector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DBoW2/FeatureVector.cpp > CMakeFiles/DBoW2_1.dir/DBoW2/FeatureVector.cpp.i

CMakeFiles/DBoW2_1.dir/DBoW2/FeatureVector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DBoW2_1.dir/DBoW2/FeatureVector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DBoW2/FeatureVector.cpp -o CMakeFiles/DBoW2_1.dir/DBoW2/FeatureVector.cpp.s

CMakeFiles/DBoW2_1.dir/DBoW2/ScoringObject.cpp.o: CMakeFiles/DBoW2_1.dir/flags.make
CMakeFiles/DBoW2_1.dir/DBoW2/ScoringObject.cpp.o: ../DBoW2/ScoringObject.cpp
CMakeFiles/DBoW2_1.dir/DBoW2/ScoringObject.cpp.o: CMakeFiles/DBoW2_1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/DBoW2_1.dir/DBoW2/ScoringObject.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/DBoW2_1.dir/DBoW2/ScoringObject.cpp.o -MF CMakeFiles/DBoW2_1.dir/DBoW2/ScoringObject.cpp.o.d -o CMakeFiles/DBoW2_1.dir/DBoW2/ScoringObject.cpp.o -c /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DBoW2/ScoringObject.cpp

CMakeFiles/DBoW2_1.dir/DBoW2/ScoringObject.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DBoW2_1.dir/DBoW2/ScoringObject.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DBoW2/ScoringObject.cpp > CMakeFiles/DBoW2_1.dir/DBoW2/ScoringObject.cpp.i

CMakeFiles/DBoW2_1.dir/DBoW2/ScoringObject.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DBoW2_1.dir/DBoW2/ScoringObject.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DBoW2/ScoringObject.cpp -o CMakeFiles/DBoW2_1.dir/DBoW2/ScoringObject.cpp.s

CMakeFiles/DBoW2_1.dir/DUtils/Random.cpp.o: CMakeFiles/DBoW2_1.dir/flags.make
CMakeFiles/DBoW2_1.dir/DUtils/Random.cpp.o: ../DUtils/Random.cpp
CMakeFiles/DBoW2_1.dir/DUtils/Random.cpp.o: CMakeFiles/DBoW2_1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/DBoW2_1.dir/DUtils/Random.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/DBoW2_1.dir/DUtils/Random.cpp.o -MF CMakeFiles/DBoW2_1.dir/DUtils/Random.cpp.o.d -o CMakeFiles/DBoW2_1.dir/DUtils/Random.cpp.o -c /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DUtils/Random.cpp

CMakeFiles/DBoW2_1.dir/DUtils/Random.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DBoW2_1.dir/DUtils/Random.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DUtils/Random.cpp > CMakeFiles/DBoW2_1.dir/DUtils/Random.cpp.i

CMakeFiles/DBoW2_1.dir/DUtils/Random.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DBoW2_1.dir/DUtils/Random.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DUtils/Random.cpp -o CMakeFiles/DBoW2_1.dir/DUtils/Random.cpp.s

CMakeFiles/DBoW2_1.dir/DUtils/Timestamp.cpp.o: CMakeFiles/DBoW2_1.dir/flags.make
CMakeFiles/DBoW2_1.dir/DUtils/Timestamp.cpp.o: ../DUtils/Timestamp.cpp
CMakeFiles/DBoW2_1.dir/DUtils/Timestamp.cpp.o: CMakeFiles/DBoW2_1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/DBoW2_1.dir/DUtils/Timestamp.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/DBoW2_1.dir/DUtils/Timestamp.cpp.o -MF CMakeFiles/DBoW2_1.dir/DUtils/Timestamp.cpp.o.d -o CMakeFiles/DBoW2_1.dir/DUtils/Timestamp.cpp.o -c /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DUtils/Timestamp.cpp

CMakeFiles/DBoW2_1.dir/DUtils/Timestamp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DBoW2_1.dir/DUtils/Timestamp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DUtils/Timestamp.cpp > CMakeFiles/DBoW2_1.dir/DUtils/Timestamp.cpp.i

CMakeFiles/DBoW2_1.dir/DUtils/Timestamp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DBoW2_1.dir/DUtils/Timestamp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/DUtils/Timestamp.cpp -o CMakeFiles/DBoW2_1.dir/DUtils/Timestamp.cpp.s

# Object files for target DBoW2_1
DBoW2_1_OBJECTS = \
"CMakeFiles/DBoW2_1.dir/DBoW2/BowVector.cpp.o" \
"CMakeFiles/DBoW2_1.dir/DBoW2/FORB.cpp.o" \
"CMakeFiles/DBoW2_1.dir/DBoW2/FeatureVector.cpp.o" \
"CMakeFiles/DBoW2_1.dir/DBoW2/ScoringObject.cpp.o" \
"CMakeFiles/DBoW2_1.dir/DUtils/Random.cpp.o" \
"CMakeFiles/DBoW2_1.dir/DUtils/Timestamp.cpp.o"

# External object files for target DBoW2_1
DBoW2_1_EXTERNAL_OBJECTS =

../lib/libDBoW2_1.so: CMakeFiles/DBoW2_1.dir/DBoW2/BowVector.cpp.o
../lib/libDBoW2_1.so: CMakeFiles/DBoW2_1.dir/DBoW2/FORB.cpp.o
../lib/libDBoW2_1.so: CMakeFiles/DBoW2_1.dir/DBoW2/FeatureVector.cpp.o
../lib/libDBoW2_1.so: CMakeFiles/DBoW2_1.dir/DBoW2/ScoringObject.cpp.o
../lib/libDBoW2_1.so: CMakeFiles/DBoW2_1.dir/DUtils/Random.cpp.o
../lib/libDBoW2_1.so: CMakeFiles/DBoW2_1.dir/DUtils/Timestamp.cpp.o
../lib/libDBoW2_1.so: CMakeFiles/DBoW2_1.dir/build.make
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_alphamat.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_barcode.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_intensity_transform.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_mcc.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_rapid.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_wechat_qrcode.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.5.4d
../lib/libDBoW2_1.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5.4d
../lib/libDBoW2_1.so: CMakeFiles/DBoW2_1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX shared library ../lib/libDBoW2_1.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/DBoW2_1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/DBoW2_1.dir/build: ../lib/libDBoW2_1.so
.PHONY : CMakeFiles/DBoW2_1.dir/build

CMakeFiles/DBoW2_1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/DBoW2_1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/DBoW2_1.dir/clean

CMakeFiles/DBoW2_1.dir/depend:
	cd /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2 /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2 /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/build /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/build /home/gina/mei/ORB_SLAM3/Thirdparty/DBoW2/build/CMakeFiles/DBoW2_1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/DBoW2_1.dir/depend

