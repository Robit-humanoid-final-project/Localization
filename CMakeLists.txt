cmake_minimum_required(VERSION 3.16)

project(MyProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets UiTools)

add_executable(MyProject
    main.cpp
    Localization.ui
    src/odometry.cpp
    src/particle.cpp
    src/mcl.cpp
    src/stadium_info.cpp
)

target_include_directories(MyProject PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(MyProject PRIVATE 
    Qt6::Widgets
    Qt6::UiTools
)