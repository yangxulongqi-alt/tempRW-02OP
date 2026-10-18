cmake_minimum_required(VERSION 3.8)
project(yesense_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# 设置 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)

# 设置头文件搜索目录，包括顶级 include 目录和子目录 include/motor_control
include_directories(
  include
  include/${PROJECT_NAME}  # 添加这一行
)

add_library(yesenseIMU STATIC 
  src/analysis_data.cpp
  src/imu_driver.cpp)

add_executable(imu_test_basic test/main.cpp)
target_link_libraries(imu_test_basic yesenseIMU)

# ---------------------------------------------------------------------------
#                            install
# ---------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS yesenseIMU
  EXPORT export_yesenseIMU
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS imu_test_basic
  DESTINATION lib/${PROJECT_NAME}/examples
)

# ---------------------------------------------------------------------------
#                            export
# ---------------------------------------------------------------------------
ament_export_include_directories(include)
ament_export_libraries(yesenseIMU)

ament_package()
