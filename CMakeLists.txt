# CMakeList.txt : CMake project for opencvboosttest, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

project ("botswarm")

find_package (OpenCV REQUIRED)
find_package (Boost REQUIRED system)
find_package (SDL2 REQUIRED)
find_package (XIMEA REQUIRED)

# pthread is necessary for the TimeoutSerial
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Add source to this project's executable.
add_executable (botswarm
    main.cpp
    Game/GameStarter.cpp
    Game/ScoreManager.cpp
    Game/Games/ChaseGame.cpp
    Game/Games/PacmanGame.cpp
    Grid/GridManager.cpp
    Grid/GridManager.hpp
    Grid/Node.cpp
    Grid/Node.hpp
    Grid/Line.cpp
    Grid/Line.hpp
    Grid/PathFinder.cpp
    Grid/PathFinder.hpp
    Threads/Thread.cpp
    Threads/CameraThread.cpp
    Threads/DetectorThread.cpp
    Threads/InputThread.cpp
    Threads/RadioThread.cpp
    Camera/Camera.cpp
    Camera/Detector.cpp
    Camera/xiApiPlusOcv.cpp
    Robot/Robot.cpp
    Misc/Time.cpp
    Misc/UnitConverter.cpp
    Radio/CommandCenter.cpp
    Radio/CommandGenerator.cpp
    Radio/TimeoutSerial.cpp
 )

target_include_directories (botswarm PUBLIC ${OpenCV_INCLUDE_DIRS} ${SDL2_INCLUDE_DIRS} ${XIMEA_INCLUDE_DIRS} )
target_link_libraries (botswarm PRIVATE ${OpenCV_LIBS} ${SDL2_LIBRARIES} ${XIMEA_LIBS} Boost::headers Boost::system /usr/lib/libm3api.so)

