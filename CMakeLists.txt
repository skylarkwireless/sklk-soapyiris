########################################################################
# Project setup for Iris SoapySDR support on local PC
########################################################################
cmake_minimum_required(VERSION 2.8)
project(SoapyIris CXX)

set(BUILD_INFO "git") #TODO git hash, or release
set(DRIVER_VERSION "2017.11.0.1-${BUILD_INFO}")
add_definitions(-DDRIVER_VERSION=\"${DRIVER_VERSION}\")

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

########################################################################
# SoapySDR dependency
########################################################################
find_package(SoapySDR "0.6" CONFIG)

if (NOT SoapySDR_FOUND)
    message(FATAL_ERROR "SoapySDR development files not found")
    return()
endif ()

message(STATUS "SoapySDR_INCLUDE_DIRS: ${SoapySDR_INCLUDE_DIRS}")
message(STATUS "SoapySDR_LIBRARIES: ${SoapySDR_LIBRARIES}")

#support threaded client code
#notice that -pthread is not the same as -lpthread
if(CMAKE_COMPILER_IS_GNUCXX)
    list(APPEND SoapySDR_LIBRARIES -pthread)
endif()

########################################################################
# companion utility
########################################################################
add_subdirectory(SoapyRemoteLibs)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/SoapyRemoteLibs)

########################################################################
# build the module
########################################################################
include_directories(${CMAKE_CURRENT_BINARY_DIR}/SoapyRemoteLibs) #socket header
if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-Wall)
    add_compile_options(-Werror)
endif()

SOAPY_SDR_MODULE_UTIL(
    TARGET IrisSupport
    SOURCES
        IfAddrsUtils.cpp
        iris_registration.cpp
        iris_settings.cpp
        iris_streaming.cpp
        iris_formats.cpp
    LIBRARIES SoapyRemoteLibs
)

########################################################################
# build tests
########################################################################
add_subdirectory(tests)

########################################################################
# Build summary
########################################################################
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Project version: ${DRIVER_VERSION}")
