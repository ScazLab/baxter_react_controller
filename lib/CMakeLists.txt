# Copyright: (C) 2016 Social Robotics Lab - Yale University
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.3)
project(baxterReactController)

# find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
# find_package(OpenCV REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

## Declare the C++ libraries
# For the record, robot_utils is decoupled from robot_interface
# so that we can link robot_perception against robot_utils only
# without the need to link against the full robot_interface lib,
# which would be an unnecessary overhead.

add_library(react_controller include/react_controller/controllerNLP.h
                             include/react_controller/ctrlThread.h
                             include/react_controller/mathUtils.h
                             include/react_controller/baxterChain.h
                             include/react_controller/avoidance.h
                             src/react_controller/controllerNLP.cpp
                             src/react_controller/ctrlThread.cpp
                             src/react_controller/mathUtils.cpp
                             src/react_controller/baxterChain.cpp
                             src/react_controller/collisionPoints.cpp
                             src/react_controller/avoidance.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

add_dependencies(react_controller    ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(react_controller    ${IPOPT_LIBRARIES}
                                          ${Eigen_LIBRARIES}
                                          ${catkin_LIBRARIES})

## Mark libraries for installation
install (TARGETS react_controller
         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY   ${PROJECT_SOURCE_DIR}/include/react_controller
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)
