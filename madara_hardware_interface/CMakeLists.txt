cmake_minimum_required(VERSION 3.8)
project(madara_hardware_interface)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake     REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib        REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ── Shared library (pluginlib plugin) ─────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/madara_uart_system.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Register pluginlib plugin
pluginlib_export_plugin_description_file(
  hardware_interface madara_hardware_interface.xml)

# ── Install ───────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  EXPORT  export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include)

install(FILES madara_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_package()