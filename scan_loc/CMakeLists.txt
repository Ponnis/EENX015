# CMakeLists files in this project can
cmake_minimum_required(VERSION 3.5)
project(scan_loc)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PhoLocalization REQUIRED CONFIG PATHS "$ENV{PHO_LOCALIZATION_PATH}")
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# find_package(rosidl_typesupport_cpp REQUIRED)
# find_package(rosidl_generator_cpp REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/TransformMatrix.msg" 
#   "srv/GetTransformMatrix.srv"
#  )


LINK_DIRECTORIES(lib)

add_executable (SimpleLocate src/SimpleLocate.cpp)

# rosidl_target_interfaces(SimpleLocate
# ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(SimpleLocate 
    ${PHOLOCALIZATION_LIBRARIES}
    ${TF2_LIBRARIES}
    ${TF2_ROS_LIBRARIES}
    ${TF2_GEOMETRY_MSGS_LIBRARIES}
)

target_include_directories (SimpleLocate PUBLIC ${PHOLOCALIZATION_INCLUDE_DIRS} ${TF2_ROS_INCLUDE_DIRS})

ament_target_dependencies(SimpleLocate rclcpp std_msgs tf2 tf2_ros tf2_geometry_msgs)

SET(LIBS PhoXi_API_msvc14_Release_1.2.14.lib PhoXi_API_msvc14_Release_1.2.14.dll 
PhoXi_API_msvc14_Debug_1.2.14.lib PhoXi_API_msvc14_Debug_1.2.14.dll
PhoLocalization_SDK__Release_1.2.2.lib PhoLocalization_SDK__Release_1.2.2.dll
PhoXi_API_dynamic_msvc12_Release_1.2.7.lib PhoXi_API_dynamic_msvc12_Release_1.2.7.dll)
# pcl_kdtree_release.dll)
# pcl_search_release.dll pcl_common_release.dll pcl_registration_release.dll python27.dll pcl_features_release.dll PhoComputerUUID__Release_1.2.2.dll)

TARGET_LINK_LIBRARIES(SimpleLocate ${LIBS} )

install(TARGETS
  SimpleLocate
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  lib/
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  lib/T-fittings
  DESTINATION include/)


# ament_export_dependencies(rosidl_default_runtime)

ament_package()