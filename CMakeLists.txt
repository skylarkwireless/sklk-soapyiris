########################################################################
# Project setup for Iris SoapySDR support on local PC
########################################################################
cmake_minimum_required(VERSION 2.8)
project(SoapyIris CXX)

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

set(CMAKE_CXX_STANDARD 11)

########################################################################
# gather version information
# packagers may specify -DIRIS_EXTVER="foo" to replace the git hash
########################################################################
file(READ "${PROJECT_SOURCE_DIR}/Changelog.txt" changelog_txt)
string(REGEX MATCH "Release ([0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+) \\(" CHANGELOG_MATCH "${changelog_txt}")
if(NOT CHANGELOG_MATCH)
    message(FATAL_ERROR "Failed to extract version number from Changelog.txt")
endif(NOT CHANGELOG_MATCH)
set(IRIS_LIBVER "${CMAKE_MATCH_1}")

if (NOT IRIS_EXTVER)
    include(${PROJECT_SOURCE_DIR}/cmake/Modules/GetGitRevisionDescription.cmake)
    get_git_head_revision(GITREFSPEC GITHASH)
    if (GITHASH)
        string(SUBSTRING "${GITHASH}" 0 8 GITHASH)
        set(IRIS_EXTVER "${GITHASH}")
    else (GITHASH)
        set(IRIS_EXTVER "unknown")
    endif (GITHASH)
endif()

set(DRIVER_VERSION "${IRIS_LIBVER}-${IRIS_EXTVER}")
add_definitions(-DDRIVER_VERSION=\"${DRIVER_VERSION}\")

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

########################################################################
# companion utility
########################################################################
add_subdirectory(SoapyRemoteLibs)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/SoapyRemoteLibs)

if (MSVC)
    list(APPEND EXTRA_NET_SOURCES GetAdaptersWindows.cpp)
else()
    list(APPEND EXTRA_NET_SOURCES IfAddrsUtils.cpp)
endif()

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
        ${EXTRA_NET_SOURCES}
        iris_registration.cpp
        iris_settings.cpp
        iris_streaming.cpp
        iris_formats.cpp
    LIBRARIES SoapyRemoteLibs
)

#support threaded client code
#notice that -pthread is not the same as -lpthread
if(CMAKE_COMPILER_IS_GNUCXX)
    target_link_libraries(IrisSupport PRIVATE -pthread)
endif()

########################################################################
# build tests
########################################################################
add_subdirectory(tests)

########################################################################
# Build summary
########################################################################
message(STATUS "${PROJECT_NAME} version: ${DRIVER_VERSION}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Project version: ${DRIVER_VERSION}")
