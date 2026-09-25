cmake_minimum_required(VERSION 3.8)
project(ethercat_interface)


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# The EcSlave plugin model (ec_slave.hpp, ec_joint_pdo_channel_manager.hpp,
# ec_sync_manager users building ec_sync_info_t) is IgH-specific: export the
# ecrt.h include directory so EcSlave plugins find it through this package.
# Header only (EC_READ_*/EC_WRITE_* macros, structs): the library stays IgH-free.
set(ETHERLAB_LOCAL_DIR /usr/local/etherlab)
find_path(ETHERCAT_INCLUDE_DIR NAMES ecrt.h PATHS ${ETHERLAB_LOCAL_DIR}/include)
if(NOT ETHERCAT_INCLUDE_DIR)
  message(WARNING "ecrt.h not found: EcSlave plugins will not compile against ethercat_interface")
endif()

add_library(
  ${PROJECT_NAME}
  SHARED
  src/ec_pdo_channel_manager.cpp
  src/ec_pdo_single_interface_channel_manager.cpp
  src/ec_pdo_group_interface_channel_manager.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

#target_link_libraries(${PROJECT_NAME} ${ETHERCAT_LIB})

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  diagnostic_msgs
)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  # find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  find_package(yaml_cpp_vendor REQUIRED)
  find_package(yaml-cpp REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # Test PdoChannelManager
  ament_add_gmock(
    test_ec_pdo_channel_manager
    test/test_ec_pdo_channel_manager.cpp
  )
  target_include_directories(test_ec_pdo_channel_manager PRIVATE include)

  target_link_libraries(test_ec_pdo_channel_manager
    ${PROJECT_NAME}
    yaml-cpp)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
if(ETHERCAT_INCLUDE_DIR)
  ament_export_include_directories(${ETHERCAT_INCLUDE_DIR})
endif()

ament_export_dependencies(
  rclcpp
  diagnostic_msgs
)
ament_package()
