cmake_minimum_required(VERSION 3.16)

project(localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets UiTools)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(localization
    src/main.cpp
    src/subscribe/imu_subscriber.cpp
    src/subscribe/ik_coord_subscriber.cpp
    src/subscribe/vision_subscriber.cpp
    Localization.ui
    src/Odometry/odometry.cpp
    src/particle.cpp
    src/mcl.cpp
    src/stadium_info.cpp
    src/publish/world_model_publisher.cpp
    src/mainwindow.cpp 
)

target_include_directories(localization PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(localization
    Qt6::Widgets
    Qt6::UiTools
    Threads::Threads
    yaml-cpp
)

ament_target_dependencies(localization
    ament_index_cpp
    rclcpp
    humanoid_interfaces
)

install(TARGETS localization
    DESTINATION lib/${PROJECT_NAME}
)

install(FILES Localization.ui stadium.png
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
