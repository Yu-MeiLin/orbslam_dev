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
CMAKE_SOURCE_DIR = /home/gina/mei/orbslam_dev/ORB_SLAM3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gina/mei/orbslam_dev/ORB_SLAM3/build

# Include any dependencies generated for this target.
include CMakeFiles/stereo_inertial_realsense_t265.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/stereo_inertial_realsense_t265.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/stereo_inertial_realsense_t265.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stereo_inertial_realsense_t265.dir/flags.make

CMakeFiles/stereo_inertial_realsense_t265.dir/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc.o: CMakeFiles/stereo_inertial_realsense_t265.dir/flags.make
CMakeFiles/stereo_inertial_realsense_t265.dir/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc.o: ../Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc
CMakeFiles/stereo_inertial_realsense_t265.dir/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc.o: CMakeFiles/stereo_inertial_realsense_t265.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gina/mei/orbslam_dev/ORB_SLAM3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stereo_inertial_realsense_t265.dir/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/stereo_inertial_realsense_t265.dir/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc.o -MF CMakeFiles/stereo_inertial_realsense_t265.dir/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc.o.d -o CMakeFiles/stereo_inertial_realsense_t265.dir/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc.o -c /home/gina/mei/orbslam_dev/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc

CMakeFiles/stereo_inertial_realsense_t265.dir/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_inertial_realsense_t265.dir/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gina/mei/orbslam_dev/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc > CMakeFiles/stereo_inertial_realsense_t265.dir/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc.i

CMakeFiles/stereo_inertial_realsense_t265.dir/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_inertial_realsense_t265.dir/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gina/mei/orbslam_dev/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc -o CMakeFiles/stereo_inertial_realsense_t265.dir/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc.s

# Object files for target stereo_inertial_realsense_t265
stereo_inertial_realsense_t265_OBJECTS = \
"CMakeFiles/stereo_inertial_realsense_t265.dir/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc.o"

# External object files for target stereo_inertial_realsense_t265
stereo_inertial_realsense_t265_EXTERNAL_OBJECTS =

../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: CMakeFiles/stereo_inertial_realsense_t265.dir/Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc.o
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: CMakeFiles/stereo_inertial_realsense_t265.dir/build.make
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: ../lib/libORB_SLAM3.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_alphamat.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_barcode.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_intensity_transform.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_mcc.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_rapid.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_wechat_qrcode.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5.4d
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /home/gina/mei/Pangolin/build/libpango_glgeometry.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /home/gina/mei/Pangolin/build/libpango_geometry.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /home/gina/mei/Pangolin/build/libpango_python.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /home/gina/mei/Pangolin/build/libpango_plot.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /home/gina/mei/Pangolin/build/libpango_scene.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /home/gina/mei/Pangolin/build/libpango_tools.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /home/gina/mei/Pangolin/build/libpango_display.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /home/gina/mei/Pangolin/build/libpango_vars.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /home/gina/mei/Pangolin/build/libpango_video.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /home/gina/mei/Pangolin/build/libpango_packetstream.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /home/gina/mei/Pangolin/build/libpango_windowing.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /home/gina/mei/Pangolin/build/libpango_opengl.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /home/gina/mei/Pangolin/build/libpango_image.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /home/gina/mei/Pangolin/build/libpango_core.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libOpenGL.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libGLX.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /home/gina/mei/Pangolin/build/libtinyobj.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: ../Thirdparty/DBoW2/lib/libDBoW2_1.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: ../Thirdparty/g2o/lib/libg2o.so
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /opt/ros/humble/lib/x86_64-linux-gnu/librealsense2.so.2.54.1
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: /opt/ros/humble/lib/x86_64-linux-gnu/librsutils.a
../Examples/Stereo-Inertial/stereo_inertial_realsense_t265: CMakeFiles/stereo_inertial_realsense_t265.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gina/mei/orbslam_dev/ORB_SLAM3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../Examples/Stereo-Inertial/stereo_inertial_realsense_t265"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereo_inertial_realsense_t265.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stereo_inertial_realsense_t265.dir/build: ../Examples/Stereo-Inertial/stereo_inertial_realsense_t265
.PHONY : CMakeFiles/stereo_inertial_realsense_t265.dir/build

CMakeFiles/stereo_inertial_realsense_t265.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stereo_inertial_realsense_t265.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stereo_inertial_realsense_t265.dir/clean

CMakeFiles/stereo_inertial_realsense_t265.dir/depend:
	cd /home/gina/mei/orbslam_dev/ORB_SLAM3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gina/mei/orbslam_dev/ORB_SLAM3 /home/gina/mei/orbslam_dev/ORB_SLAM3 /home/gina/mei/orbslam_dev/ORB_SLAM3/build /home/gina/mei/orbslam_dev/ORB_SLAM3/build /home/gina/mei/orbslam_dev/ORB_SLAM3/build/CMakeFiles/stereo_inertial_realsense_t265.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stereo_inertial_realsense_t265.dir/depend

