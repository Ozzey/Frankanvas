cmake_minimum_required(VERSION 3.16.0)
project(msrm_utils
    LANGUAGES CXX
    )

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wreturn-type")
set(CMAKE_DEBUG_POSTFIX _dbg)
set(CMAKE_EXPORT_COMPILE_COMMANDS On)
set(CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/../third_party/lib/cmake)

find_package(nlohmann_json REQUIRED)

find_program(FPING fping)
if(NOT FPING)
    message(FATAL_ERROR "fping not found!")
endif()

SET(USER_INCLUDE_PATH "" CACHE STRING "Additional header location.")
option(BUILD_TESTS "Build tests" ON)
option(USE_PCH "Use precompiled headers" OFF)
option(STATIC_LIB "Compile a static library" ON)

set(SOURCES src/conversion.cpp src/files.cpp src/json.cpp src/math.cpp src/network.cpp src/output.cpp src/system.cpp src/benchmarking.cpp src/geometry.cpp)
set(HEADER include/msrm_utils/conversion.hpp include/msrm_utils/files.hpp include/msrm_utils/json.hpp include/msrm_utils/math.hpp
    include/msrm_utils/network.hpp include/msrm_utils/output.hpp include/msrm_utils/system.hpp include/msrm_utils/benchmarking.hpp include/msrm_utils/geometry.hpp)

if(STATIC_LIB)
    add_library(msrm_cpp_utils STATIC ${SOURCES})
else()
    add_library(msrm_cpp_utils SHARED ${SOURCES})
endif()

if(USER_INCLUDE_PATH)
target_include_directories(msrm_cpp_utils PUBLIC ${USER_INCLUDE_PATH})
endif()

target_include_directories(msrm_cpp_utils PUBLIC include ${CMAKE_SOURCE_DIR}/../third_party/include)
link_directories(msrm_cpp_utils ${CMAKE_SOURCE_DIR}/../third_party/lib)
target_link_libraries(msrm_cpp_utils ssl z)

set_target_properties(msrm_cpp_utils PROPERTIES PUBLIC_HEADER "${HEADER}")
set_target_properties(msrm_cpp_utils PROPERTIES UNITY_BUILD ON)

if(BUILD_TESTS)
  enable_testing()
  add_executable(unit_tests src/unit_test.cpp unit_tests/test_conversion.cpp unit_tests/test_network.cpp
      unit_tests/test_files.cpp unit_tests/test_system.cpp unit_tests/test_math.cpp unit_tests/test_benchmarking.cpp unit_tests/test_geometry.cpp)
  target_link_libraries(unit_tests msrm_cpp_utils boost_system pthread crypto)
  set_target_properties(unit_tests PROPERTIES UNITY_BUILD ON)
  if(USE_PCH)
    target_precompile_headers(unit_tests PRIVATE ${CMAKE_SOURCE_DIR}/include/commons/pch.hpp)
  endif()
endif()

if(USE_PCH)
  target_precompile_headers(msrm_cpp_utils PRIVATE ${CMAKE_SOURCE_DIR}/include/commons/pch.hpp)
endif()

install(TARGETS msrm_cpp_utils CONFIGURATIONS Release DESTINATION ${CMAKE_INSTALL_PREFIX}/lib PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include/msrm_utils)
install(TARGETS msrm_cpp_utils CONFIGURATIONS Debug DESTINATION ${CMAKE_INSTALL_PREFIX}/lib PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include/msrm_utils)
