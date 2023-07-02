# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /mnt/c/Users/pgian/Documents/final_HMMY/trafficFluid/MultiagentMCTSController

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/c/Users/pgian/Documents/final_HMMY/trafficFluid/MultiagentMCTSController

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /mnt/c/Users/pgian/Documents/final_HMMY/trafficFluid/MultiagentMCTSController/CMakeFiles /mnt/c/Users/pgian/Documents/final_HMMY/trafficFluid/MultiagentMCTSController/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /mnt/c/Users/pgian/Documents/final_HMMY/trafficFluid/MultiagentMCTSController/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named LaneFreePlugin

# Build rule for target.
LaneFreePlugin: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 LaneFreePlugin
.PHONY : LaneFreePlugin

# fast build rule for target.
LaneFreePlugin/fast:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/build
.PHONY : LaneFreePlugin/fast

src/Controller.o: src/Controller.cpp.o

.PHONY : src/Controller.o

# target to build an object file
src/Controller.cpp.o:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/Controller.cpp.o
.PHONY : src/Controller.cpp.o

src/Controller.i: src/Controller.cpp.i

.PHONY : src/Controller.i

# target to preprocess a source file
src/Controller.cpp.i:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/Controller.cpp.i
.PHONY : src/Controller.cpp.i

src/Controller.s: src/Controller.cpp.s

.PHONY : src/Controller.s

# target to generate assembly for a file
src/Controller.cpp.s:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/Controller.cpp.s
.PHONY : src/Controller.cpp.s

src/LaneFree.o: src/LaneFree.cpp.o

.PHONY : src/LaneFree.o

# target to build an object file
src/LaneFree.cpp.o:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/LaneFree.cpp.o
.PHONY : src/LaneFree.cpp.o

src/LaneFree.i: src/LaneFree.cpp.i

.PHONY : src/LaneFree.i

# target to preprocess a source file
src/LaneFree.cpp.i:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/LaneFree.cpp.i
.PHONY : src/LaneFree.cpp.i

src/LaneFree.s: src/LaneFree.cpp.s

.PHONY : src/LaneFree.s

# target to generate assembly for a file
src/LaneFree.cpp.s:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/LaneFree.cpp.s
.PHONY : src/LaneFree.cpp.s

src/MCTS.o: src/MCTS.cpp.o

.PHONY : src/MCTS.o

# target to build an object file
src/MCTS.cpp.o:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/MCTS.cpp.o
.PHONY : src/MCTS.cpp.o

src/MCTS.i: src/MCTS.cpp.i

.PHONY : src/MCTS.i

# target to preprocess a source file
src/MCTS.cpp.i:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/MCTS.cpp.i
.PHONY : src/MCTS.cpp.i

src/MCTS.s: src/MCTS.cpp.s

.PHONY : src/MCTS.s

# target to generate assembly for a file
src/MCTS.cpp.s:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/MCTS.cpp.s
.PHONY : src/MCTS.cpp.s

src/MaxPlus.o: src/MaxPlus.cpp.o

.PHONY : src/MaxPlus.o

# target to build an object file
src/MaxPlus.cpp.o:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/MaxPlus.cpp.o
.PHONY : src/MaxPlus.cpp.o

src/MaxPlus.i: src/MaxPlus.cpp.i

.PHONY : src/MaxPlus.i

# target to preprocess a source file
src/MaxPlus.cpp.i:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/MaxPlus.cpp.i
.PHONY : src/MaxPlus.cpp.i

src/MaxPlus.s: src/MaxPlus.cpp.s

.PHONY : src/MaxPlus.s

# target to generate assembly for a file
src/MaxPlus.cpp.s:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/MaxPlus.cpp.s
.PHONY : src/MaxPlus.cpp.s

src/PotentialField.o: src/PotentialField.cpp.o

.PHONY : src/PotentialField.o

# target to build an object file
src/PotentialField.cpp.o:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/PotentialField.cpp.o
.PHONY : src/PotentialField.cpp.o

src/PotentialField.i: src/PotentialField.cpp.i

.PHONY : src/PotentialField.i

# target to preprocess a source file
src/PotentialField.cpp.i:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/PotentialField.cpp.i
.PHONY : src/PotentialField.cpp.i

src/PotentialField.s: src/PotentialField.cpp.s

.PHONY : src/PotentialField.s

# target to generate assembly for a file
src/PotentialField.cpp.s:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/PotentialField.cpp.s
.PHONY : src/PotentialField.cpp.s

src/laneFreeGlobalState.o: src/laneFreeGlobalState.cpp.o

.PHONY : src/laneFreeGlobalState.o

# target to build an object file
src/laneFreeGlobalState.cpp.o:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/laneFreeGlobalState.cpp.o
.PHONY : src/laneFreeGlobalState.cpp.o

src/laneFreeGlobalState.i: src/laneFreeGlobalState.cpp.i

.PHONY : src/laneFreeGlobalState.i

# target to preprocess a source file
src/laneFreeGlobalState.cpp.i:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/laneFreeGlobalState.cpp.i
.PHONY : src/laneFreeGlobalState.cpp.i

src/laneFreeGlobalState.s: src/laneFreeGlobalState.cpp.s

.PHONY : src/laneFreeGlobalState.s

# target to generate assembly for a file
src/laneFreeGlobalState.cpp.s:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/laneFreeGlobalState.cpp.s
.PHONY : src/laneFreeGlobalState.cpp.s

src/utils.o: src/utils.cpp.o

.PHONY : src/utils.o

# target to build an object file
src/utils.cpp.o:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/utils.cpp.o
.PHONY : src/utils.cpp.o

src/utils.i: src/utils.cpp.i

.PHONY : src/utils.i

# target to preprocess a source file
src/utils.cpp.i:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/utils.cpp.i
.PHONY : src/utils.cpp.i

src/utils.s: src/utils.cpp.s

.PHONY : src/utils.s

# target to generate assembly for a file
src/utils.cpp.s:
	$(MAKE) -f CMakeFiles/LaneFreePlugin.dir/build.make CMakeFiles/LaneFreePlugin.dir/src/utils.cpp.s
.PHONY : src/utils.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... LaneFreePlugin"
	@echo "... src/Controller.o"
	@echo "... src/Controller.i"
	@echo "... src/Controller.s"
	@echo "... src/LaneFree.o"
	@echo "... src/LaneFree.i"
	@echo "... src/LaneFree.s"
	@echo "... src/MCTS.o"
	@echo "... src/MCTS.i"
	@echo "... src/MCTS.s"
	@echo "... src/MaxPlus.o"
	@echo "... src/MaxPlus.i"
	@echo "... src/MaxPlus.s"
	@echo "... src/PotentialField.o"
	@echo "... src/PotentialField.i"
	@echo "... src/PotentialField.s"
	@echo "... src/laneFreeGlobalState.o"
	@echo "... src/laneFreeGlobalState.i"
	@echo "... src/laneFreeGlobalState.s"
	@echo "... src/utils.o"
	@echo "... src/utils.i"
	@echo "... src/utils.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

