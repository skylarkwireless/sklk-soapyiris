include_directories(${SoapySDR_INCLUDE_DIRS})
add_executable(IrisFullDuplex IrisFullDuplex.cpp)
target_link_libraries(IrisFullDuplex ${SoapySDR_LIBRARIES})
