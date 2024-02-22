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
CMAKE_SOURCE_DIR = /home/gina/mei/orbslam_dev/slam/src/orbslam3_ros2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gina/mei/orbslam_dev/slam/build/orbslam3

# Include any dependencies generated for this target.
include CMakeFiles/stereo.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/stereo.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/stereo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stereo.dir/flags.make

CMakeFiles/stereo.dir/src/stereo/stereo.cpp.o: CMakeFiles/stereo.dir/flags.make
CMakeFiles/stereo.dir/src/stereo/stereo.cpp.o: /home/gina/mei/orbslam_dev/slam/src/orbslam3_ros2/src/stereo/stereo.cpp
CMakeFiles/stereo.dir/src/stereo/stereo.cpp.o: CMakeFiles/stereo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gina/mei/orbslam_dev/slam/build/orbslam3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stereo.dir/src/stereo/stereo.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/stereo.dir/src/stereo/stereo.cpp.o -MF CMakeFiles/stereo.dir/src/stereo/stereo.cpp.o.d -o CMakeFiles/stereo.dir/src/stereo/stereo.cpp.o -c /home/gina/mei/orbslam_dev/slam/src/orbslam3_ros2/src/stereo/stereo.cpp

CMakeFiles/stereo.dir/src/stereo/stereo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo.dir/src/stereo/stereo.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gina/mei/orbslam_dev/slam/src/orbslam3_ros2/src/stereo/stereo.cpp > CMakeFiles/stereo.dir/src/stereo/stereo.cpp.i

CMakeFiles/stereo.dir/src/stereo/stereo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo.dir/src/stereo/stereo.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gina/mei/orbslam_dev/slam/src/orbslam3_ros2/src/stereo/stereo.cpp -o CMakeFiles/stereo.dir/src/stereo/stereo.cpp.s

CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.o: CMakeFiles/stereo.dir/flags.make
CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.o: /home/gina/mei/orbslam_dev/slam/src/orbslam3_ros2/src/stereo/stereo-slam-node.cpp
CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.o: CMakeFiles/stereo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gina/mei/orbslam_dev/slam/build/orbslam3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.o -MF CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.o.d -o CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.o -c /home/gina/mei/orbslam_dev/slam/src/orbslam3_ros2/src/stereo/stereo-slam-node.cpp

CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gina/mei/orbslam_dev/slam/src/orbslam3_ros2/src/stereo/stereo-slam-node.cpp > CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.i

CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gina/mei/orbslam_dev/slam/src/orbslam3_ros2/src/stereo/stereo-slam-node.cpp -o CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.s

# Object files for target stereo
stereo_OBJECTS = \
"CMakeFiles/stereo.dir/src/stereo/stereo.cpp.o" \
"CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.o"

# External object files for target stereo
stereo_EXTERNAL_OBJECTS =

stereo: CMakeFiles/stereo.dir/src/stereo/stereo.cpp.o
stereo: CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.o
stereo: CMakeFiles/stereo.dir/build.make
stereo: /opt/ros/humble/lib/libcv_bridge.so
stereo: /opt/ros/humble/lib/libmessage_filters.so
stereo: /home/gina/mei/orbslam_dev/ORB_SLAM3/lib/libORB_SLAM3.so
stereo: /home/gina/mei/orbslam_dev/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
stereo: /home/gina/mei/orbslam_dev/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
stereo: /home/gina/mei/Pangolin/build/libpango_glgeometry.so
stereo: /home/gina/mei/Pangolin/build/libpango_python.so
stereo: /home/gina/mei/Pangolin/build/libpango_scene.so
stereo: /home/gina/mei/Pangolin/build/libpango_tools.so
stereo: /home/gina/mei/Pangolin/build/libpango_video.so
stereo: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_alphamat.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_barcode.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_intensity_transform.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_mcc.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_rapid.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_wechat_qrcode.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.5.4d
stereo: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_fastrtps_c.so
stereo: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_c.so
stereo: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so
stereo: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_fastrtps_cpp.so
stereo: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_cpp.so
stereo: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_cpp.so
stereo: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
stereo: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
stereo: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
stereo: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
stereo: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
stereo: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
stereo: /opt/ros/humble/lib/libsensor_msgs__rosidl_generator_py.so
stereo: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_c.so
stereo: /opt/ros/humble/lib/libsensor_msgs__rosidl_generator_c.so
stereo: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_py.so
stereo: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_c.so
stereo: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_c.so
stereo: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_py.so
stereo: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_c.so
stereo: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_c.so
stereo: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_cpp.so
stereo: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
stereo: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_cpp.so
stereo: /opt/ros/humble/lib/librclcpp.so
stereo: /opt/ros/humble/lib/liblibstatistics_collector.so
stereo: /opt/ros/humble/lib/librcl.so
stereo: /opt/ros/humble/lib/librmw_implementation.so
stereo: /opt/ros/humble/lib/libament_index_cpp.so
stereo: /opt/ros/humble/lib/librcl_logging_spdlog.so
stereo: /opt/ros/humble/lib/librcl_logging_interface.so
stereo: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so
stereo: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
stereo: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so
stereo: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
stereo: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_cpp.so
stereo: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_py.so
stereo: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_c.so
stereo: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_c.so
stereo: /opt/ros/humble/lib/librcl_yaml_param_parser.so
stereo: /opt/ros/humble/lib/libyaml.so
stereo: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.so
stereo: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.so
stereo: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
stereo: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
stereo: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
stereo: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_py.so
stereo: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_c.so
stereo: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_c.so
stereo: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_c.so
stereo: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.so
stereo: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_c.so
stereo: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_cpp.so
stereo: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.so
stereo: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_cpp.so
stereo: /opt/ros/humble/lib/libfastcdr.so.1.0.24
stereo: /opt/ros/humble/lib/librmw.so
stereo: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
stereo: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
stereo: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
stereo: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
stereo: /opt/ros/humble/lib/librosidl_typesupport_introspection_cpp.so
stereo: /opt/ros/humble/lib/librosidl_typesupport_introspection_c.so
stereo: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
stereo: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
stereo: /opt/ros/humble/lib/librosidl_typesupport_cpp.so
stereo: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_py.so
stereo: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_py.so
stereo: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_c.so
stereo: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
stereo: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_c.so
stereo: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_c.so
stereo: /opt/ros/humble/lib/librosidl_typesupport_c.so
stereo: /opt/ros/humble/lib/librcpputils.so
stereo: /opt/ros/humble/lib/librosidl_runtime_c.so
stereo: /usr/lib/x86_64-linux-gnu/libpython3.10.so
stereo: /opt/ros/humble/lib/libtracetools.so
stereo: /opt/ros/humble/lib/librcutils.so
stereo: /home/gina/mei/Pangolin/build/libpango_geometry.so
stereo: /home/gina/mei/Pangolin/build/libtinyobj.so
stereo: /home/gina/mei/Pangolin/build/libpango_plot.so
stereo: /home/gina/mei/Pangolin/build/libpango_display.so
stereo: /home/gina/mei/Pangolin/build/libpango_vars.so
stereo: /home/gina/mei/Pangolin/build/libpango_windowing.so
stereo: /home/gina/mei/Pangolin/build/libpango_opengl.so
stereo: /usr/lib/x86_64-linux-gnu/libGLEW.so
stereo: /usr/lib/x86_64-linux-gnu/libOpenGL.so
stereo: /usr/lib/x86_64-linux-gnu/libGLX.so
stereo: /usr/lib/x86_64-linux-gnu/libGLU.so
stereo: /home/gina/mei/Pangolin/build/libpango_image.so
stereo: /home/gina/mei/Pangolin/build/libpango_packetstream.so
stereo: /home/gina/mei/Pangolin/build/libpango_core.so
stereo: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.5.4d
stereo: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5.4d
stereo: CMakeFiles/stereo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gina/mei/orbslam_dev/slam/build/orbslam3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable stereo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stereo.dir/build: stereo
.PHONY : CMakeFiles/stereo.dir/build

CMakeFiles/stereo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stereo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stereo.dir/clean

CMakeFiles/stereo.dir/depend:
	cd /home/gina/mei/orbslam_dev/slam/build/orbslam3 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gina/mei/orbslam_dev/slam/src/orbslam3_ros2 /home/gina/mei/orbslam_dev/slam/src/orbslam3_ros2 /home/gina/mei/orbslam_dev/slam/build/orbslam3 /home/gina/mei/orbslam_dev/slam/build/orbslam3 /home/gina/mei/orbslam_dev/slam/build/orbslam3/CMakeFiles/stereo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stereo.dir/depend

