# Install script for directory: /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl" TYPE DIRECTORY FILES "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/resources" REGEX "/\\.DS\\_Store$" EXCLUDE)
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "omplapp")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "omplapp")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl" TYPE DIRECTORY FILES "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/webapp" REGEX "/\\.DS\\_Store$" EXCLUDE)
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "omplapp")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE RENAME "ompl.pc" FILES "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/CMakeModules/ompl.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl" TYPE FILE RENAME "ompl-config.cmake" FILES "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/doc/markdown/FindOMPL.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl" TYPE FILE FILES "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ompl-config-version.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl" TYPE FILE FILES "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/ompl.conf")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ompl/doc/cmake_install.cmake")
  INCLUDE("/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ompl/py-bindings/cmake_install.cmake")
  INCLUDE("/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ompl/src/cmake_install.cmake")
  INCLUDE("/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ompl/tests/cmake_install.cmake")
  INCLUDE("/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ompl/demos/cmake_install.cmake")
  INCLUDE("/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ompl/scripts/cmake_install.cmake")
  INCLUDE("/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/gui/cmake_install.cmake")
  INCLUDE("/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/doc/cmake_install.cmake")
  INCLUDE("/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/src/cmake_install.cmake")
  INCLUDE("/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/demos/cmake_install.cmake")
  INCLUDE("/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/benchmark/cmake_install.cmake")
  INCLUDE("/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/py-bindings/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
