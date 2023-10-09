cmake_minimum_required(VERSION 2.8.12)
project(mkfilter_catkin)

find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS include/
    LIBRARIES ${PROJECT_NAME}
)

#Include directories
include_directories(src include ${catkin_INCLUDE_DIRS})

add_compile_options(-std=c++11)

add_library(${PROJECT_NAME}
    src/complex.cpp
    src/mkfilter.cpp
    src/MKFilter.cpp
)

if (CATKIN_ENABLE_TESTING)
    add_executable(testMKFilter test/testMKFilter.cpp)
    target_link_libraries(testMKFilter
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
    )
endif (CATKIN_ENABLE_TESTING)

#Install rules
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

