# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "D:\Program Files\JetBrains\CLion 2020.2.1\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "D:\Program Files\JetBrains\CLion 2020.2.1\bin\cmake\win\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\John\CLionProjects\5519_BugHW2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\John\CLionProjects\5519_BugHW2\cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/5519_BugHW2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/5519_BugHW2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/5519_BugHW2.dir/flags.make

CMakeFiles/5519_BugHW2.dir/main.cpp.obj: CMakeFiles/5519_BugHW2.dir/flags.make
CMakeFiles/5519_BugHW2.dir/main.cpp.obj: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\John\CLionProjects\5519_BugHW2\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/5519_BugHW2.dir/main.cpp.obj"
	"D:\Program Files (x86)\mingw-w64\i686-8.1.0-posix-dwarf-rt_v6-rev0\mingw32\bin\g++.exe"  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\5519_BugHW2.dir\main.cpp.obj -c C:\Users\John\CLionProjects\5519_BugHW2\main.cpp

CMakeFiles/5519_BugHW2.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/5519_BugHW2.dir/main.cpp.i"
	"D:\Program Files (x86)\mingw-w64\i686-8.1.0-posix-dwarf-rt_v6-rev0\mingw32\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\John\CLionProjects\5519_BugHW2\main.cpp > CMakeFiles\5519_BugHW2.dir\main.cpp.i

CMakeFiles/5519_BugHW2.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/5519_BugHW2.dir/main.cpp.s"
	"D:\Program Files (x86)\mingw-w64\i686-8.1.0-posix-dwarf-rt_v6-rev0\mingw32\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\John\CLionProjects\5519_BugHW2\main.cpp -o CMakeFiles\5519_BugHW2.dir\main.cpp.s

# Object files for target 5519_BugHW2
5519_BugHW2_OBJECTS = \
"CMakeFiles/5519_BugHW2.dir/main.cpp.obj"

# External object files for target 5519_BugHW2
5519_BugHW2_EXTERNAL_OBJECTS =

5519_BugHW2.exe: CMakeFiles/5519_BugHW2.dir/main.cpp.obj
5519_BugHW2.exe: CMakeFiles/5519_BugHW2.dir/build.make
5519_BugHW2.exe: CMakeFiles/5519_BugHW2.dir/linklibs.rsp
5519_BugHW2.exe: CMakeFiles/5519_BugHW2.dir/objects1.rsp
5519_BugHW2.exe: CMakeFiles/5519_BugHW2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\John\CLionProjects\5519_BugHW2\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable 5519_BugHW2.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\5519_BugHW2.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/5519_BugHW2.dir/build: 5519_BugHW2.exe

.PHONY : CMakeFiles/5519_BugHW2.dir/build

CMakeFiles/5519_BugHW2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\5519_BugHW2.dir\cmake_clean.cmake
.PHONY : CMakeFiles/5519_BugHW2.dir/clean

CMakeFiles/5519_BugHW2.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\John\CLionProjects\5519_BugHW2 C:\Users\John\CLionProjects\5519_BugHW2 C:\Users\John\CLionProjects\5519_BugHW2\cmake-build-debug C:\Users\John\CLionProjects\5519_BugHW2\cmake-build-debug C:\Users\John\CLionProjects\5519_BugHW2\cmake-build-debug\CMakeFiles\5519_BugHW2.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/5519_BugHW2.dir/depend

