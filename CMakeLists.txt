cmake_minimum_required(VERSION 2.8)
project(ExplodedView)


set(CMAKE_INCLUDE_CURRENT_DIR ON)


set(SOURCE_FILES
        src/pluginMain.cpp
        src/ExplodedView.h
        src/ExplodedView.cpp )

# Sorry, this is hardcoded
set(CMAKE_MODULE_PATH /media/miller/BAGHEERA/cadaver/CMAKE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Maya REQUIRED)

include_directories(${MAYA_INCLUDE_DIR})
link_directories(${MAYA_LIBRARY_DIR})

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${MAYA_LIBRARIES})

MAYA_PLUGIN(${PROJECT_NAME})