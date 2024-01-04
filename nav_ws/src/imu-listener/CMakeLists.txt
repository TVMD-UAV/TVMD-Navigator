cmake_minimum_required(VERSION 3.8)
project(imu-listener)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<depend> REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libs/libi2c)

add_executable(imu-listener src/imu-listener.cpp)
target_include_directories(imu-listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Add i2c library
target_include_directories(imu-listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs/libi2c/include>
  $<INSTALL_INTERFACE:libs/libi2c/include>)

target_compile_features(imu-listener PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(${PROJECT_NAME} PUBLIC i2c)

install(TARGETS imu-listener
  DESTINATION lib/${PROJECT_NAME})

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
