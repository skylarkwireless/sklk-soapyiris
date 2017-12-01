########################################################################
# Copy of some libs from SoapyRemote common/ for socket support
########################################################################

enable_language(C)

list(APPEND COMMON_SOURCES
    SoapyURLUtils.cpp
    SoapyRPCSocket.cpp
)

#check for platform-specific network header
include(CheckIncludeFiles)
CHECK_INCLUDE_FILES(winsock2.h HAS_WINSOCK2_H)
CHECK_INCLUDE_FILES(ws2tcpip.h HAS_WS2TCPIP_H)
CHECK_INCLUDE_FILES(netdb.h HAS_NETDB_H)
CHECK_INCLUDE_FILES(unistd.h HAS_UNISTD_H)
CHECK_INCLUDE_FILES(netinet/in.h HAS_NETINET_IN_H)
CHECK_INCLUDE_FILES(netinet/tcp.h HAS_NETINET_TCP_H)
CHECK_INCLUDE_FILES(sys/types.h HAS_SYS_TYPES_H)
CHECK_INCLUDE_FILES(sys/socket.h HAS_SYS_SOCKET_H)
CHECK_INCLUDE_FILES(arpa/inet.h HAS_ARPA_INET_H)
CHECK_INCLUDE_FILES(ifaddrs.h HAS_IFADDRS_H)
CHECK_INCLUDE_FILES(net/if.h HAS_NET_IF_H)
CHECK_INCLUDE_FILES(fcntl.h HAS_FCNTL_H)

# Thread config support
find_library(
    RT_LIBRARIES
    NAMES rt
    PATHS /usr/lib /usr/lib64
)

if (RT_LIBRARIES)
    list(APPEND EXTRA_LIBRARIES ${RT_LIBRARIES})
endif()

if(WIN32)
    list(APPEND COMMON_SOURCES ThreadPrioWindows.cpp)
elseif(UNIX)
    list(APPEND COMMON_SOURCES ThreadPrioUnix.cpp)
endif()

#network libraries
if (WIN32)
    list(APPEND EXTRA_LIBRARIES ws2_32)
endif (WIN32)

#create private include header for network compatibility
include_directories(${CMAKE_CURRENT_BINARY_DIR})
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/SoapySocketDefs.in.hpp
    ${CMAKE_CURRENT_BINARY_DIR}/SoapySocketDefs.hpp)

#build a static library
add_library(SoapyRemoteLibs STATIC ${COMMON_SOURCES})
if(EXTRA_LIBRARIES)
    target_link_libraries(SoapyRemoteLibs ${EXTRA_LIBRARIES})
endif()
set_property(TARGET SoapyRemoteLibs PROPERTY POSITION_INDEPENDENT_CODE TRUE)
