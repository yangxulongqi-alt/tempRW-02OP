cmake_minimum_required(VERSION 3.8)
project(nezha_lqr_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(encos_eth_master REQUIRED)
find_package(yesense_imu REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}  # 添加这一行
)

# ---------------------------------------------------------------------------
#                            exe gen
#  wblqr_controller, lqr控制器
# ---------------------------------------------------------------------------
# 添加可执行文件（指定链接依赖库）
add_executable(wblqr_controller 
    src/lqrController.cpp
    src/lqrKTable.cpp
    src/vmc.cpp
    src/main.cpp)
ament_target_dependencies(wblqr_controller
  rclcpp sensor_msgs geometry_msgs
  tf2_ros tf2
  encos_eth_master
  yesense_imu
  rosidl_typesupport_cpp
)
target_include_directories(wblqr_controller PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(wblqr_controller "${TORCH_LIBRARIES}")
# 对当前目标：O3优化+启用浮点运算优化（编译时间变长）
target_compile_options(wblqr_controller PRIVATE -O3 -ffast-math)

# ---------------------------------------------------------------------------
#                            install
# ---------------------------------------------------------------------------
# 安装Target
install(TARGETS 
  wblqr_controller
  DESTINATION lib/${PROJECT_NAME}
)
# 安装launch目录、配置文件
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# # ---------------------------------------------------------------------------
# #                             setcap
# # ---------------------------------------------------------------------------
set(WB_CONTROLLER_PATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/wblqr_controller")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
  install(CODE "
    message(STATUS \"Setting network capabilities for ${WB_CONTROLLER_PATH}...\")
    execute_process(
      COMMAND sudo setcap cap_net_admin,cap_net_raw=eip ${WB_CONTROLLER_PATH}
      RESULT_VARIABLE result
      OUTPUT_VARIABLE output
      ERROR_VARIABLE error
    )
    if(result)
      message(FATAL_ERROR \"Failed to setcap: ${error}\")
    else()
      execute_process(
        COMMAND getcap ${WB_CONTROLLER_PATH}
        OUTPUT_VARIABLE cap_output
      )
      message(STATUS \"Successfully set capabilities: ${cap_output}\")
    endif()
  ")
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
