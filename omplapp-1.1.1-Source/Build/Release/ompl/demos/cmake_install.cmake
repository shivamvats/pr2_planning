# Install script for directory: /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE FILE FILES
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/ThunderLightning.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/RigidBodyPlanning.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/CForestCircleGridBenchmark.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/KinematicChainBenchmark.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/HybridSystemPlanning.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/RigidBodyPlanningWithIntegrationAndControls.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/TriangulationDemo.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/LTLWithTriangulation.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/PlannerProgressProperties.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/HypercubeBenchmark.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/StateSampling.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/OptimalPlanning.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/Point2DPlanning.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/PlannerData.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/RigidBodyPlanningWithIK.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/RigidBodyPlanningWithODESolverAndControls.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/OpenDERigidBodyPlanning.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/RigidBodyPlanningWithControls.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/GeometricCarPlanning.cpp"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/PlannerData.py"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/RandomWalkPlanner.py"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/RigidBodyPlanningWithODESolverAndControls.py"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/StateSampling.py"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/Point2DPlanning.py"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/OptimalPlanning.py"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/RigidBodyPlanning.py"
    "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/RigidBodyPlanningWithControls.py"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE DIRECTORY FILES "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/demos/Koules")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")

