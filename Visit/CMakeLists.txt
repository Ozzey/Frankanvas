cmake_minimum_required(VERSION 3.16)
project(Drawing_Demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(MSRM_UTILS_DIR "${CMAKE_CURRENT_LIST_DIR}/../third_party/msrm_utils" CACHE PATH "Path to msrm_utils")

find_package( Boost REQUIRED COMPONENTS filesystem)
# set(glm_DIR ${CMAKE_SOURCE_DIR}/glm/cmake/glm) # if necessary
# find_package(glm REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
#find_package(CGAL)

if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
else()
    message(ERROR
        "The Boost library headers were not found; targets depending "
        "on Boost won't be available.")
endif()

#set(EIGEN3_INCLUDE_DIRS /home/panda/Documents/eigen)

SET(MESSAGE_PARSER new_message_parser/bvh.h new_message_parser/joint.h new_message_parser/utils.h new_message_parser/bvh-parser.h)


#MESSAGE(${EIGEN3_INCLUDE_DIRS})
MESSAGE(${MSRM_UTILS_DIR})
include_directories(
#${EIGEN3_INCLUDE_DIRS}
                    ${MSRM_UTILS_DIR}/cpp/include
                    ${MSRM_UTILS_DIR}/third_party/include
                    UDPServer
                    rate_limiter
                    include)

link_directories(${MSRM_UTILS_DIR}/third_party/lib
                 ${MSRM_UTILS_DIR}/cpp/build)

add_library(cppserver
    CppServer/source/server/asio/service.cpp
    CppServer/source/server/asio/timer.cpp
    CppServer/source/server/asio/udp_server.cpp)

target_include_directories(cppserver PUBLIC
    include
    CppServer/include)

target_link_libraries(cppserver PUBLIC
    Boost::filesystem
    boost_system
    OpenSSL::SSL
    OpenSSL::Crypto
    Threads::Threads)

add_library(msrm_cpp_utils STATIC
    ${MSRM_UTILS_DIR}/cpp/src/benchmarking.cpp
    ${MSRM_UTILS_DIR}/cpp/src/conversion.cpp
    ${MSRM_UTILS_DIR}/cpp/src/files.cpp
    ${MSRM_UTILS_DIR}/cpp/src/geometry.cpp
    ${MSRM_UTILS_DIR}/cpp/src/json.cpp
    ${MSRM_UTILS_DIR}/cpp/src/math.cpp
    ${MSRM_UTILS_DIR}/cpp/src/network.cpp
    ${MSRM_UTILS_DIR}/cpp/src/output.cpp
    ${MSRM_UTILS_DIR}/cpp/src/system.cpp)

target_include_directories(msrm_cpp_utils PUBLIC
    ${MSRM_UTILS_DIR}/cpp/include
    ${MSRM_UTILS_DIR}/third_party/include)

target_link_libraries(msrm_cpp_utils PUBLIC
    boost_system
    OpenSSL::SSL
    OpenSSL::Crypto
    Threads::Threads
    z)

set(Telepresence telepresence_interface/MiosEventServer.cpp telepresence_interface/utils.cpp telepresence_interface/miosInterface.cpp rate_limiter/rate_limiting.cpp)

set(MASTER_LIBRARIES msrm_cpp_utils pthread ssl crypto boost_system z)

add_executable(Drawing_Demo main.cpp ${Telepresence} ${MESSAGE_PARSER})

target_link_libraries(Drawing_Demo ${MASTER_LIBRARIES} cppserver Eigen3::Eigen) #CGAL::CGAL)
