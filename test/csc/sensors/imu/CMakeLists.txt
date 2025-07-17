add_executable(test_imu_task
  ${CMAKE_SOURCE_DIR}/src/csc/sensors/imu/imu_task.cpp
  test_imu_task.cpp
  imu_stub.cpp
)

target_compile_definitions(test_imu_task PRIVATE UNIT_TESTING)

target_include_directories(test_imu_task PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/external/protocore/include
  ${CMAKE_SOURCE_DIR}/external/kfplusplus/include
)

target_link_libraries(test_imu_task PRIVATE protocore kfplusplus gtest gtest_main)

add_test(NAME test_imu_task COMMAND test_imu_task)
