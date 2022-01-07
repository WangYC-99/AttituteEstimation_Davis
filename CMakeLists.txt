cmake_minimum_required (VERSION 2.6)
project (Davis2021)

set(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_COMPILER /usr/bin/g++)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
IF (OpenCV_VERSION VERSION_LESS "3.1.0")
    MESSAGE(FATAL_ERROR "Cannot find OpenCV 3.1.0 or newer.")
ENDIF()

find_package(Boost COMPONENTS regex system REQUIRED)


include_directories(./include)

add_library(hardware ./src/camera.cpp)
target_link_libraries(hardware caer)

set(DAVIS_LIB hardware ${OpenCV_LIBS} ${Boost_LIBRARIES})

ADD_EXECUTABLE(davis_cvgui main.cpp)
TARGET_LINK_LIBRARIES(davis_cvgui ${DAVIS_LIB})

ADD_EXECUTABLE(distort ./src/distortTest.cpp)
TARGET_LINK_LIBRARIES(distort ${DAVIS_LIB})

add_executable(calib ./calib/CalibMono.cpp)
target_link_libraries(calib ${DAVIS_LIB})

add_executable(takePhoto ./calib/TakePhoto.cpp)
target_link_libraries(takePhoto ${DAVIS_LIB})