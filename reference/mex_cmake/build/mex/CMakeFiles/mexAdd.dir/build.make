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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/gary/mex_cmake/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gary/mex_cmake/build

# Include any dependencies generated for this target.
include mex/CMakeFiles/mexAdd.dir/depend.make

# Include the progress variables for this target.
include mex/CMakeFiles/mexAdd.dir/progress.make

# Include the compile flags for this target's objects.
include mex/CMakeFiles/mexAdd.dir/flags.make

mex/CMakeFiles/mexAdd.dir/mexAdd.cpp.o: mex/CMakeFiles/mexAdd.dir/flags.make
mex/CMakeFiles/mexAdd.dir/mexAdd.cpp.o: /home/gary/mex_cmake/src/mex/mexAdd.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/gary/mex_cmake/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object mex/CMakeFiles/mexAdd.dir/mexAdd.cpp.o"
	cd /home/gary/mex_cmake/build/mex && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mexAdd.dir/mexAdd.cpp.o -c /home/gary/mex_cmake/src/mex/mexAdd.cpp

mex/CMakeFiles/mexAdd.dir/mexAdd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mexAdd.dir/mexAdd.cpp.i"
	cd /home/gary/mex_cmake/build/mex && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/gary/mex_cmake/src/mex/mexAdd.cpp > CMakeFiles/mexAdd.dir/mexAdd.cpp.i

mex/CMakeFiles/mexAdd.dir/mexAdd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mexAdd.dir/mexAdd.cpp.s"
	cd /home/gary/mex_cmake/build/mex && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/gary/mex_cmake/src/mex/mexAdd.cpp -o CMakeFiles/mexAdd.dir/mexAdd.cpp.s

mex/CMakeFiles/mexAdd.dir/mexAdd.cpp.o.requires:
.PHONY : mex/CMakeFiles/mexAdd.dir/mexAdd.cpp.o.requires

mex/CMakeFiles/mexAdd.dir/mexAdd.cpp.o.provides: mex/CMakeFiles/mexAdd.dir/mexAdd.cpp.o.requires
	$(MAKE) -f mex/CMakeFiles/mexAdd.dir/build.make mex/CMakeFiles/mexAdd.dir/mexAdd.cpp.o.provides.build
.PHONY : mex/CMakeFiles/mexAdd.dir/mexAdd.cpp.o.provides

mex/CMakeFiles/mexAdd.dir/mexAdd.cpp.o.provides.build: mex/CMakeFiles/mexAdd.dir/mexAdd.cpp.o

# Object files for target mexAdd
mexAdd_OBJECTS = \
"CMakeFiles/mexAdd.dir/mexAdd.cpp.o"

# External object files for target mexAdd
mexAdd_EXTERNAL_OBJECTS =

mex/mexAdd.mexa64: mex/CMakeFiles/mexAdd.dir/mexAdd.cpp.o
mex/mexAdd.mexa64: mex/CMakeFiles/mexAdd.dir/build.make
mex/mexAdd.mexa64: /usr/local/MATLAB/R2015a/bin/glnxa64/libmex.so
mex/mexAdd.mexa64: /usr/local/MATLAB/R2015a/bin/glnxa64/libmx.so
mex/mexAdd.mexa64: /home/gary/mex_cmake/src/Matlabdef.def
mex/mexAdd.mexa64: mex/CMakeFiles/mexAdd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library mexAdd.mexa64"
	cd /home/gary/mex_cmake/build/mex && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mexAdd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
mex/CMakeFiles/mexAdd.dir/build: mex/mexAdd.mexa64
.PHONY : mex/CMakeFiles/mexAdd.dir/build

mex/CMakeFiles/mexAdd.dir/requires: mex/CMakeFiles/mexAdd.dir/mexAdd.cpp.o.requires
.PHONY : mex/CMakeFiles/mexAdd.dir/requires

mex/CMakeFiles/mexAdd.dir/clean:
	cd /home/gary/mex_cmake/build/mex && $(CMAKE_COMMAND) -P CMakeFiles/mexAdd.dir/cmake_clean.cmake
.PHONY : mex/CMakeFiles/mexAdd.dir/clean

mex/CMakeFiles/mexAdd.dir/depend:
	cd /home/gary/mex_cmake/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gary/mex_cmake/src /home/gary/mex_cmake/src/mex /home/gary/mex_cmake/build /home/gary/mex_cmake/build/mex /home/gary/mex_cmake/build/mex/CMakeFiles/mexAdd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : mex/CMakeFiles/mexAdd.dir/depend

