# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd-build

# Include any dependencies generated for this target.
include CMakeFiles/ccd_static.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ccd_static.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ccd_static.dir/flags.make

CMakeFiles/ccd_static.dir/src/ccd.c.o: CMakeFiles/ccd_static.dir/flags.make
CMakeFiles/ccd_static.dir/src/ccd.c.o: /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/ccd.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd-build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/ccd_static.dir/src/ccd.c.o"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/ccd_static.dir/src/ccd.c.o   -c /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/ccd.c

CMakeFiles/ccd_static.dir/src/ccd.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ccd_static.dir/src/ccd.c.i"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/ccd.c > CMakeFiles/ccd_static.dir/src/ccd.c.i

CMakeFiles/ccd_static.dir/src/ccd.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ccd_static.dir/src/ccd.c.s"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/ccd.c -o CMakeFiles/ccd_static.dir/src/ccd.c.s

CMakeFiles/ccd_static.dir/src/ccd.c.o.requires:
.PHONY : CMakeFiles/ccd_static.dir/src/ccd.c.o.requires

CMakeFiles/ccd_static.dir/src/ccd.c.o.provides: CMakeFiles/ccd_static.dir/src/ccd.c.o.requires
	$(MAKE) -f CMakeFiles/ccd_static.dir/build.make CMakeFiles/ccd_static.dir/src/ccd.c.o.provides.build
.PHONY : CMakeFiles/ccd_static.dir/src/ccd.c.o.provides

CMakeFiles/ccd_static.dir/src/ccd.c.o.provides.build: CMakeFiles/ccd_static.dir/src/ccd.c.o

CMakeFiles/ccd_static.dir/src/mpr.c.o: CMakeFiles/ccd_static.dir/flags.make
CMakeFiles/ccd_static.dir/src/mpr.c.o: /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/mpr.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd-build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/ccd_static.dir/src/mpr.c.o"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/ccd_static.dir/src/mpr.c.o   -c /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/mpr.c

CMakeFiles/ccd_static.dir/src/mpr.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ccd_static.dir/src/mpr.c.i"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/mpr.c > CMakeFiles/ccd_static.dir/src/mpr.c.i

CMakeFiles/ccd_static.dir/src/mpr.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ccd_static.dir/src/mpr.c.s"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/mpr.c -o CMakeFiles/ccd_static.dir/src/mpr.c.s

CMakeFiles/ccd_static.dir/src/mpr.c.o.requires:
.PHONY : CMakeFiles/ccd_static.dir/src/mpr.c.o.requires

CMakeFiles/ccd_static.dir/src/mpr.c.o.provides: CMakeFiles/ccd_static.dir/src/mpr.c.o.requires
	$(MAKE) -f CMakeFiles/ccd_static.dir/build.make CMakeFiles/ccd_static.dir/src/mpr.c.o.provides.build
.PHONY : CMakeFiles/ccd_static.dir/src/mpr.c.o.provides

CMakeFiles/ccd_static.dir/src/mpr.c.o.provides.build: CMakeFiles/ccd_static.dir/src/mpr.c.o

CMakeFiles/ccd_static.dir/src/polytope.c.o: CMakeFiles/ccd_static.dir/flags.make
CMakeFiles/ccd_static.dir/src/polytope.c.o: /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/polytope.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd-build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/ccd_static.dir/src/polytope.c.o"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/ccd_static.dir/src/polytope.c.o   -c /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/polytope.c

CMakeFiles/ccd_static.dir/src/polytope.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ccd_static.dir/src/polytope.c.i"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/polytope.c > CMakeFiles/ccd_static.dir/src/polytope.c.i

CMakeFiles/ccd_static.dir/src/polytope.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ccd_static.dir/src/polytope.c.s"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/polytope.c -o CMakeFiles/ccd_static.dir/src/polytope.c.s

CMakeFiles/ccd_static.dir/src/polytope.c.o.requires:
.PHONY : CMakeFiles/ccd_static.dir/src/polytope.c.o.requires

CMakeFiles/ccd_static.dir/src/polytope.c.o.provides: CMakeFiles/ccd_static.dir/src/polytope.c.o.requires
	$(MAKE) -f CMakeFiles/ccd_static.dir/build.make CMakeFiles/ccd_static.dir/src/polytope.c.o.provides.build
.PHONY : CMakeFiles/ccd_static.dir/src/polytope.c.o.provides

CMakeFiles/ccd_static.dir/src/polytope.c.o.provides.build: CMakeFiles/ccd_static.dir/src/polytope.c.o

CMakeFiles/ccd_static.dir/src/support.c.o: CMakeFiles/ccd_static.dir/flags.make
CMakeFiles/ccd_static.dir/src/support.c.o: /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/support.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd-build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/ccd_static.dir/src/support.c.o"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/ccd_static.dir/src/support.c.o   -c /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/support.c

CMakeFiles/ccd_static.dir/src/support.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ccd_static.dir/src/support.c.i"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/support.c > CMakeFiles/ccd_static.dir/src/support.c.i

CMakeFiles/ccd_static.dir/src/support.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ccd_static.dir/src/support.c.s"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/support.c -o CMakeFiles/ccd_static.dir/src/support.c.s

CMakeFiles/ccd_static.dir/src/support.c.o.requires:
.PHONY : CMakeFiles/ccd_static.dir/src/support.c.o.requires

CMakeFiles/ccd_static.dir/src/support.c.o.provides: CMakeFiles/ccd_static.dir/src/support.c.o.requires
	$(MAKE) -f CMakeFiles/ccd_static.dir/build.make CMakeFiles/ccd_static.dir/src/support.c.o.provides.build
.PHONY : CMakeFiles/ccd_static.dir/src/support.c.o.provides

CMakeFiles/ccd_static.dir/src/support.c.o.provides.build: CMakeFiles/ccd_static.dir/src/support.c.o

CMakeFiles/ccd_static.dir/src/vec3.c.o: CMakeFiles/ccd_static.dir/flags.make
CMakeFiles/ccd_static.dir/src/vec3.c.o: /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/vec3.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd-build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/ccd_static.dir/src/vec3.c.o"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/ccd_static.dir/src/vec3.c.o   -c /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/vec3.c

CMakeFiles/ccd_static.dir/src/vec3.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ccd_static.dir/src/vec3.c.i"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/vec3.c > CMakeFiles/ccd_static.dir/src/vec3.c.i

CMakeFiles/ccd_static.dir/src/vec3.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ccd_static.dir/src/vec3.c.s"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd/src/vec3.c -o CMakeFiles/ccd_static.dir/src/vec3.c.s

CMakeFiles/ccd_static.dir/src/vec3.c.o.requires:
.PHONY : CMakeFiles/ccd_static.dir/src/vec3.c.o.requires

CMakeFiles/ccd_static.dir/src/vec3.c.o.provides: CMakeFiles/ccd_static.dir/src/vec3.c.o.requires
	$(MAKE) -f CMakeFiles/ccd_static.dir/build.make CMakeFiles/ccd_static.dir/src/vec3.c.o.provides.build
.PHONY : CMakeFiles/ccd_static.dir/src/vec3.c.o.provides

CMakeFiles/ccd_static.dir/src/vec3.c.o.provides.build: CMakeFiles/ccd_static.dir/src/vec3.c.o

# Object files for target ccd_static
ccd_static_OBJECTS = \
"CMakeFiles/ccd_static.dir/src/ccd.c.o" \
"CMakeFiles/ccd_static.dir/src/mpr.c.o" \
"CMakeFiles/ccd_static.dir/src/polytope.c.o" \
"CMakeFiles/ccd_static.dir/src/support.c.o" \
"CMakeFiles/ccd_static.dir/src/vec3.c.o"

# External object files for target ccd_static
ccd_static_EXTERNAL_OBJECTS =

libccd.a: CMakeFiles/ccd_static.dir/src/ccd.c.o
libccd.a: CMakeFiles/ccd_static.dir/src/mpr.c.o
libccd.a: CMakeFiles/ccd_static.dir/src/polytope.c.o
libccd.a: CMakeFiles/ccd_static.dir/src/support.c.o
libccd.a: CMakeFiles/ccd_static.dir/src/vec3.c.o
libccd.a: CMakeFiles/ccd_static.dir/build.make
libccd.a: CMakeFiles/ccd_static.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C static library libccd.a"
	$(CMAKE_COMMAND) -P CMakeFiles/ccd_static.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ccd_static.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ccd_static.dir/build: libccd.a
.PHONY : CMakeFiles/ccd_static.dir/build

CMakeFiles/ccd_static.dir/requires: CMakeFiles/ccd_static.dir/src/ccd.c.o.requires
CMakeFiles/ccd_static.dir/requires: CMakeFiles/ccd_static.dir/src/mpr.c.o.requires
CMakeFiles/ccd_static.dir/requires: CMakeFiles/ccd_static.dir/src/polytope.c.o.requires
CMakeFiles/ccd_static.dir/requires: CMakeFiles/ccd_static.dir/src/support.c.o.requires
CMakeFiles/ccd_static.dir/requires: CMakeFiles/ccd_static.dir/src/vec3.c.o.requires
.PHONY : CMakeFiles/ccd_static.dir/requires

CMakeFiles/ccd_static.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ccd_static.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ccd_static.dir/clean

CMakeFiles/ccd_static.dir/depend:
	cd /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd-build /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd-build /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ccd-prefix/src/ccd-build/CMakeFiles/ccd_static.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ccd_static.dir/depend

