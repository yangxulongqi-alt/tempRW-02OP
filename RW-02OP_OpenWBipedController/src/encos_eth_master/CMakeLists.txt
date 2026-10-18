cmake_minimum_required(VERSION 3.8)
project(encos_eth_master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# debug
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "选择构建类型: Debug, Release, RelWithDebInfo, MinSizeRel" FORCE)
  # 设置构建类型选项，可通过 cmake-gui 或 ccmake 选择
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

add_subdirectory(thirdparty)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)

# ---------------------------------------------------------------------------
#                            lib
# ---------------------------------------------------------------------------
file(GLOB SOURCES  src/*.c  src/*.cpp)

# motor库
add_library(libmotor STATIC ${SOURCES})

target_include_directories(libmotor
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${Boost_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/soem/soem
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/soem/osal
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/soem/osal/linux
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/soem/oshw/linux
)
target_link_libraries(libmotor PUBLIC soem m)


add_executable(motor_test_basic test/main.cpp)
target_link_libraries(motor_test_basic libmotor soem m)
target_compile_options(motor_test_basic PRIVATE
    $<$<CONFIG:Debug>:-g3 -O0 -fno-omit-frame-pointer>  # 禁用优化并保留栈帧
)
get_target_property(FLAGS motor_test_basic COMPILE_OPTIONS)
# message(WARNING "motor_test_basic COMPILE_OPTIONS: ${FLAGS}")
ament_target_dependencies(motor_test_basic 
  rclcpp
)


# ---------------------------------------------------------------------------
#                            install
#  soem最好独立包装成一个ros包，现在先这样
# ---------------------------------------------------------------------------
file(GLOB SOEM_HEADERS thirdparty/soem/soem/*.h)
file(GLOB OSAL_HEADERS thirdparty/soem/osal/osal.h thirdparty/soem/osal/linux/*.h)
file(GLOB OSHW_HEADERS thirdparty/soem/oshw/linux/*.h)
# 安装库头文件 vs soem头文件
install(
  DIRECTORY include/
  DESTINATION include
)
install(FILES
  ${SOEM_HEADERS}
  ${OSAL_HEADERS}
  ${OSHW_HEADERS}
  DESTINATION include
)

# message(WARNING " SOEM头文件 = ${SOEM_HEADERS})")
# message(WARNING " SOEM头文件 = ${ORIGIN}/../../../lib")

# 安装lib文件
install(
  TARGETS libmotor soem
  EXPORT export_libmotor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS motor_test_basic
  DESTINATION lib/${PROJECT_NAME}
)
# 定义安装后的完整路径
set(MOTOR_TEST_PATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/motor_test_basic")

# 注意：需要设置colcon对setcap的权限，colcon不会主动向用户要求权限
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
  install(CODE "
    message(STATUS \"Setting network capabilities for ${MOTOR_TEST_PATH}...\")
    execute_process(
      COMMAND sudo setcap cap_net_admin,cap_net_raw=eip ${MOTOR_TEST_PATH}
      RESULT_VARIABLE result
      OUTPUT_VARIABLE output
      ERROR_VARIABLE error
    )
    if(result)
      message(FATAL_ERROR \"Failed to setcap: ${error}\")
    else()
      execute_process(
        COMMAND getcap ${MOTOR_TEST_PATH}
        OUTPUT_VARIABLE cap_output
      )
      message(STATUS \"Successfully set capabilities: ${cap_output}\")
    endif()
  ")
endif()

# ---------------------------------------------------------------------------
#                            export
# ---------------------------------------------------------------------------
# ament包导出设置
ament_export_include_directories(include)
ament_export_libraries(libmotor)
ament_export_libraries(soem)
# ament_export_dependencies(Boost)


ament_package()


# Extra
set_target_properties(motor_test_basic PROPERTIES
  INSTALL_RPATH "/opt/ros/humble/lib"
  BUILD_RPATH   "/opt/ros/humble/lib"
)