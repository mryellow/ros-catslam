cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include(FindPkgConfig)
if (PKG_CONFIG_FOUND AND UNIX AND NOT APPLE)
   pkg_check_modules(GTK gtk+-2.0)
   if (GTK_FOUND)
      include_directories(${GTK_INCLUDE_DIRS})
      link_directories(${GTK_LIBRARY_DIRS})
      add_definitions(-DHAVE_GTK)
      # Hack around gdk-pixbuf move issues for now
      # http://ubuntuforums.org/showthread.php?t=1596930
      include_directories(/usr/include/gdk-pixbuf-2.0)
   endif(GTK_FOUND)
endif(PKG_CONFIG_FOUND AND UNIX AND NOT APPLE)

rosbuild_add_boost_directories()

find_package(OpenCV)

# Nodelet library
rosbuild_add_library(image_view src/nodelets/image_nodelet.cpp src/nodelets/disparity_nodelet.cpp src/nodelets/window_thread.cpp)
target_link_libraries(image_view ${GTK_LIBRARIES})
target_link_libraries(image_view
                      ${OpenCV_LIBRARIES}
)


# Image viewers
rosbuild_add_executable(image_view_exe src/nodes/image_view.cpp)
SET_TARGET_PROPERTIES(image_view_exe PROPERTIES OUTPUT_NAME image_view)
target_link_libraries(image_view_exe
                      ${OpenCV_LIBRARIES}
)



rosbuild_add_executable(disparity_view src/nodes/disparity_view.cpp)
target_link_libraries(disparity_view
                      ${OpenCV_LIBRARIES}
)


rosbuild_add_executable(stereo_view src/nodes/stereo_view.cpp)
target_link_libraries(stereo_view ${GTK_LIBRARIES})
rosbuild_link_boost(stereo_view signals)
target_link_libraries(stereo_view
                      ${OpenCV_LIBRARIES}
)


# Extra tools (unstable)
rosbuild_add_executable(extract_images src/nodes/extract_images.cpp)
target_link_libraries(extract_images
                      ${OpenCV_LIBRARIES}
)

rosbuild_add_executable(extract_thermal src/nodes/extract_thermal.cpp)
target_link_libraries(extract_thermal
                      ${OpenCV_LIBRARIES}
)

rosbuild_add_executable(image_saver src/nodes/image_saver.cpp)
target_link_libraries(image_saver
                      ${OpenCV_LIBRARIES}
)

