# Install script for directory: /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "omplapp")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE FILE FILES
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/SE3RigidBodyPlanning//CollisionCheckers.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/SE3RigidBodyPlanning//QuadrotorPlanning.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/SE3RigidBodyPlanning//SE3RigidBodyPlanning.py"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/SE3RigidBodyPlanning//SE3MultiRigidBodyPlanning.py"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/SE3RigidBodyPlanning//SE3MultiRigidBodyPlanning.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/SE3RigidBodyPlanning//SE3RigidBodyPlanningWithOptimization.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/SE3RigidBodyPlanning//SE3RigidBodyPlanning.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/SE3RigidBodyPlanning//SE3RigidBodyPlanningBenchmark.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/SE3RigidBodyPlanning//BlimpPlanning.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/SE2RigidBodyPlanning//DynamicCarPlanning.py"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/SE2RigidBodyPlanning//KinematicCarPlanning.py"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/SE2RigidBodyPlanning//SE2MultiRigidBodyPlanning.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/SE2RigidBodyPlanning//SE2RigidBodyPlanning.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/SE2RigidBodyPlanning//DynamicCarPlanning.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/SE2RigidBodyPlanning//KinematicCarPlanning.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/OpenDEPlanning//displayOpenDE.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/OpenDEPlanning//OMPLEnvironment.inc"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/OpenDEPlanning//displayOpenDE.h"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/OpenDEPlanning//OpenDEWorld.inc"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/OpenDEPlanning//planOpenDE.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/demos/OpenDEPlanning//OMPLSetup.inc"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "omplapp")

