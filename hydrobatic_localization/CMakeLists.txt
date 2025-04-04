cmake_minimum_required(VERSION 3.8)
project(hydrobatic_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sam_msgs REQUIRED)
find_package(smarc_msgs REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_library(CPPUNITLITE_LIBRARY NAMES CppUnitLite PATHS /usr/local/lib)

# GTSAM stuff
find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
# CppTest
#include the headers
include_directories(${PROJECT_SOURCE_DIR}/include/)
# include launch files
include_directories(${PROJECT_SOURCE_DIR}/launch/)
#include GeographicLib
include_directories(${GeographicLib_INCLUDE_DIRS})
include_directories(${Python3_INCLUDE_DIRS})
# set python interpreter path 
list(APPEND PYTHONPATH "${CMAKE_INSTALL_PREFIX}/lib/hydrobatic_localization/SAM_casadi.py")



# add_executable(hydrobatic_localization src/main.cpp src/BarometerFactor.cpp src/DvlFactor.cpp src/SamMotionModelFactor.cpp)
# ament_target_dependencies(hydrobatic_localization rclcpp sensor_msgs
#                            smarc_msgs std_msgs nav_msgs sam_msgs sam_msgs tf2 tf2_ros tf2_geometry_msgs geometry_msgs)
# target_link_libraries(hydrobatic_localization gtsam Eigen3::Eigen GeographicLib::GeographicLib)

add_executable(state_estimator src/state_estimator.cpp src/gtsam_graph.cpp src/BarometerFactor.cpp src/DvlFactor.cpp src/SamMotionModelFactor.cpp)
ament_target_dependencies(state_estimator
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  sam_msgs
  smarc_msgs
  tf2 
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)
target_link_libraries(state_estimator
  gtsam
  Eigen3::Eigen
  GeographicLib::GeographicLib
  )

# Insatll the executablese, headers and launch files
install(TARGETS
  # hydrobatic_localization
  state_estimator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


install(FILES ${PROJECT_SOURCE_DIR}/smarc_modelling/src/smarc_modelling/vehicles/SAM_casadi.py
DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  add_executable(test_motion_model_factor
    test/SamMotionModelFactorTest.cpp
    src/SamMotionModelFactor.cpp
  )
  add_executable(test_barometer_factor
    test/BarometerFactorTest.cpp
    src/BarometerFactor.cpp
  )
  add_executable(test_dvl_factor
    test/DvlFactorTest.cpp
    src/DvlFactor.cpp
  )
  # TODO inlcude CppUnitLite headers in a better way
  target_include_directories(test_barometer_factor PUBLIC /usr/local/include)
  find_library(CPPUNITLITE_LIBRARY NAMES cppunitlite PATHS /usr/local/lib)
  if(NOT CPPUNITLITE_LIBRARY)
    message(FATAL_ERROR "Could not find the CppUnitLite library in /usr/local/lib")
  endif()
  target_link_libraries(test_dvl_factor
    gtsam
    ${CPPUNITLITE_LIBRARY}
  )
  target_link_libraries(test_barometer_factor
    gtsam
    ${CPPUNITLITE_LIBRARY}
  )
  target_link_libraries(test_motion_model_factor
    gtsam
    ${CPPUNITLITE_LIBRARY}
  )
  ament_target_dependencies(test_motion_model_factor
    rclcpp
  )
  ament_add_test(test_dvl_factor COMMAND test_dvl_factor)
  ament_add_test(test_barometer_factor COMMAND test_barometer_factor)
  ament_add_test(test_motion_model_factor COMMAND test_motion_model_factor)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
