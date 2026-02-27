cmake_minimum_required(VERSION 3.16)
project(piper_gripper LANGUAGES CXX)

option(PIPER_GRIPPER_BUILD_TESTS "Build tests" ON)
option(PIPER_GRIPPER_BUILD_SHARED "Build shared library" ON)
option(PIPER_GRIPPER_BUILD_EXAMPLES "Build examples" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(PIPER_GRIPPER_SOURCES
  src/gripper_client.cpp
  src/internal/can_socket.cpp
  src/internal/protocol_codec.cpp
  src/internal/fps_counter.cpp
)

add_library(piper_gripper STATIC ${PIPER_GRIPPER_SOURCES})
target_include_directories(piper_gripper
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_options(piper_gripper PRIVATE -Wall -Wextra -Wpedantic)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(piper_gripper PRIVATE -Werror)
endif()

if(PIPER_GRIPPER_BUILD_SHARED)
  add_library(piper_gripper_shared SHARED ${PIPER_GRIPPER_SOURCES})
  set_target_properties(piper_gripper_shared PROPERTIES OUTPUT_NAME piper_gripper)
  target_include_directories(piper_gripper_shared
    PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  target_compile_options(piper_gripper_shared PRIVATE -Wall -Wextra -Wpedantic)
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(piper_gripper_shared PRIVATE -Werror)
  endif()
endif()

if(PIPER_GRIPPER_BUILD_TESTS)
  enable_testing()

  add_executable(test_protocol_codec tests/unit/test_protocol_codec.cpp)
  target_link_libraries(test_protocol_codec PRIVATE piper_gripper)
  target_include_directories(test_protocol_codec PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  add_test(NAME unit_protocol_codec COMMAND test_protocol_codec)

  add_executable(test_vcan_gripper_loop tests/integration/test_vcan_gripper_loop.cpp)
  target_link_libraries(test_vcan_gripper_loop PRIVATE piper_gripper)
  target_include_directories(test_vcan_gripper_loop PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  add_test(NAME integration_vcan_gripper_loop COMMAND test_vcan_gripper_loop)
endif()

if(PIPER_GRIPPER_BUILD_EXAMPLES)
  add_executable(test_gripper_cpp examples/test_gripper_cpp.cpp)
  target_link_libraries(test_gripper_cpp PRIVATE piper_gripper)

  add_executable(test_gripper_effort_cpp examples/test_gripper_effort_cpp.cpp)
  target_link_libraries(test_gripper_effort_cpp PRIVATE piper_gripper)
endif()
