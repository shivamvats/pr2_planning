# This file will be configured to contain variables for CPack. These variables
# should be set in the CMake list file of the project before CPack module is
# included. Example variables are:
#   CPACK_GENERATOR                     - Generator used to create package
#   CPACK_INSTALL_CMAKE_PROJECTS        - For each project (path, name, component)
#   CPACK_CMAKE_GENERATOR               - CMake Generator used for the projects
#   CPACK_INSTALL_COMMANDS              - Extra commands to install components
#   CPACK_INSTALLED_DIRECTORIES           - Extra directories to install
#   CPACK_PACKAGE_DESCRIPTION_FILE      - Description file for the package
#   CPACK_PACKAGE_DESCRIPTION_SUMMARY   - Summary of the package
#   CPACK_PACKAGE_EXECUTABLES           - List of pairs of executables and labels
#   CPACK_PACKAGE_FILE_NAME             - Name of the package generated
#   CPACK_PACKAGE_ICON                  - Icon used for the package
#   CPACK_PACKAGE_INSTALL_DIRECTORY     - Name of directory for the installer
#   CPACK_PACKAGE_NAME                  - Package project name
#   CPACK_PACKAGE_VENDOR                - Package project vendor
#   CPACK_PACKAGE_VERSION               - Package project version
#   CPACK_PACKAGE_VERSION_MAJOR         - Package project version (major)
#   CPACK_PACKAGE_VERSION_MINOR         - Package project version (minor)
#   CPACK_PACKAGE_VERSION_PATCH         - Package project version (patch)

# There are certain generator specific ones

# NSIS Generator:
#   CPACK_PACKAGE_INSTALL_REGISTRY_KEY  - Name of the registry key for the installer
#   CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS - Extra commands used during uninstall
#   CPACK_NSIS_EXTRA_INSTALL_COMMANDS   - Extra commands used during install


SET(CPACK_BINARY_BUNDLE "")
SET(CPACK_BINARY_CYGWIN "")
SET(CPACK_BINARY_DEB "")
SET(CPACK_BINARY_DRAGNDROP "")
SET(CPACK_BINARY_NSIS "")
SET(CPACK_BINARY_OSXX11 "")
SET(CPACK_BINARY_PACKAGEMAKER "")
SET(CPACK_BINARY_RPM "")
SET(CPACK_BINARY_STGZ "")
SET(CPACK_BINARY_TBZ2 "")
SET(CPACK_BINARY_TGZ "")
SET(CPACK_BINARY_TZ "")
SET(CPACK_BINARY_ZIP "")
SET(CPACK_CMAKE_GENERATOR "Unix Makefiles")
SET(CPACK_COMPONENTS_ALL "ompl;python;morse;omplapp")
SET(CPACK_COMPONENTS_ALL_SET_BY_USER "TRUE")
SET(CPACK_COMPONENT_MORSE_DEPENDS "python")
SET(CPACK_COMPONENT_MORSE_DESCRIPTION "The Blender/MORSE plugin allows one to plan paths using the MORSE robot simulator. MORSE is built on top of Blender and uses its built-in physics engine to compute physically realistic motions.")
SET(CPACK_COMPONENT_MORSE_DISPLAY_NAME "Blender/MORSE plugin")
SET(CPACK_COMPONENT_OMPLAPP_DEPENDS "python")
SET(CPACK_COMPONENT_OMPLAPP_DESCRIPTION "The OMPL.app library makes it easy to read meshes (using the Assimp library) and perform collision checking (using the FCL library). The GUI is built on top of this and makes it easy to perform geometric and control-based planning for rigid bodies.")
SET(CPACK_COMPONENT_OMPLAPP_DISPLAY_NAME "OMPL.app library and GUI")
SET(CPACK_COMPONENT_OMPL_DISPLAY_NAME "OMPL library, headers, and demos")
SET(CPACK_COMPONENT_OMPL_REQUIRED "TRUE")
SET(CPACK_COMPONENT_PYTHON_DEPENDS "ompl")
SET(CPACK_COMPONENT_PYTHON_DISPLAY_NAME "Python bindings")
SET(CPACK_COMPONENT_UNSPECIFIED_HIDDEN "TRUE")
SET(CPACK_COMPONENT_UNSPECIFIED_REQUIRED "TRUE")
SET(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "python2.7, libboost-all-dev, python-opengl, python-qt4-dev, python-qt4-gl, freeglut3-dev, libode-dev, libassimp-dev, libtriangle-dev, libccd-dev")
SET(CPACK_GENERATOR "TGZ;ZIP")
SET(CPACK_IGNORE_FILES "/.hg;/build/;.pyc$;.pyo$;__pycache__;.so$;.dylib$;.orig$;.log$;.DS_Store;/html/;/bindings/;TODO;/external/assimp;/pqp-1.3;releaseChecklist.txt;exposed_decl.pypp.txt;ompl.pc$;installPyPlusPlus.bat$;installPyPlusPlus.sh$;create_symlinks.sh$;uninstall_symlinks.sh$;config.h$;.registered$;.tar.gz;.tgz$;.zip$;download.md$;mainpage.md$;binding_generator.py$")
SET(CPACK_INSTALLED_DIRECTORIES "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source;/")
SET(CPACK_INSTALL_CMAKE_PROJECTS "")
SET(CPACK_INSTALL_PREFIX "/usr/local")
SET(CPACK_MODULE_PATH "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/CMakeModules;/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/CMakeModules")
SET(CPACK_NSIS_DISPLAY_NAME "omplapp 1.1.1")
SET(CPACK_NSIS_INSTALLER_ICON_CODE "")
SET(CPACK_NSIS_INSTALLER_MUI_ICON_CODE "")
SET(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES")
SET(CPACK_NSIS_PACKAGE_NAME "omplapp 1.1.1")
SET(CPACK_OUTPUT_CONFIG_FILE "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/CPackConfig.cmake")
SET(CPACK_PACKAGE_CONTACT "Mark Moll <mmoll@rice.edu>")
SET(CPACK_PACKAGE_DEFAULT_LOCATION "/")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "/usr/share/cmake-2.8/Templates/CPack.GenericDescription.txt")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Open Motion Planning Library (OMPL) & front-end (OMPL-APP)")
SET(CPACK_PACKAGE_FILE_NAME "omplapp-1.1.1-Source")
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "omplapp 1.1.1")
SET(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "omplapp 1.1.1")
SET(CPACK_PACKAGE_NAME "omplapp")
SET(CPACK_PACKAGE_RELOCATABLE "true")
SET(CPACK_PACKAGE_VENDOR "Rice University")
SET(CPACK_PACKAGE_VERSION "1.1.1")
SET(CPACK_PACKAGE_VERSION_MAJOR "1")
SET(CPACK_PACKAGE_VERSION_MINOR "1")
SET(CPACK_PACKAGE_VERSION_PATCH "1")
SET(CPACK_RESOURCE_FILE_LICENSE "/usr/share/cmake-2.8/Templates/CPack.GenericLicense.txt")
SET(CPACK_RESOURCE_FILE_README "/usr/share/cmake-2.8/Templates/CPack.GenericDescription.txt")
SET(CPACK_RESOURCE_FILE_WELCOME "/usr/share/cmake-2.8/Templates/CPack.GenericWelcome.txt")
SET(CPACK_RSRC_DIR "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/CMakeModules")
SET(CPACK_SET_DESTDIR "OFF")
SET(CPACK_SOURCE_CYGWIN "")
SET(CPACK_SOURCE_GENERATOR "TGZ;ZIP")
SET(CPACK_SOURCE_IGNORE_FILES "/.hg;/build/;.pyc$;.pyo$;__pycache__;.so$;.dylib$;.orig$;.log$;.DS_Store;/html/;/bindings/;TODO;/external/assimp;/pqp-1.3;releaseChecklist.txt;exposed_decl.pypp.txt;ompl.pc$;installPyPlusPlus.bat$;installPyPlusPlus.sh$;create_symlinks.sh$;uninstall_symlinks.sh$;config.h$;.registered$;.tar.gz;.tgz$;.zip$;download.md$;mainpage.md$;binding_generator.py$")
SET(CPACK_SOURCE_INSTALLED_DIRECTORIES "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source;/")
SET(CPACK_SOURCE_OUTPUT_CONFIG_FILE "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/CPackSourceConfig.cmake")
SET(CPACK_SOURCE_PACKAGE_FILE_NAME "omplapp-1.1.1-Source")
SET(CPACK_SOURCE_TBZ2 "")
SET(CPACK_SOURCE_TGZ "")
SET(CPACK_SOURCE_TOPLEVEL_TAG "Linux-Source")
SET(CPACK_SOURCE_TZ "")
SET(CPACK_SOURCE_ZIP "")
SET(CPACK_STRIP_FILES "")
SET(CPACK_SYSTEM_NAME "Linux")
SET(CPACK_TOPLEVEL_TAG "Linux-Source")
