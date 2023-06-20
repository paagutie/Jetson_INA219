cmake_minimum_required(VERSION 3.5)
project(Jetson_INA219)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


include_directories(include)

file(GLOB SOURCE_FILES
 include/INA219.h
 include/INA219.cpp
)

add_executable(${PROJECT_NAME} src/main.cpp ${SOURCE_FILES})

