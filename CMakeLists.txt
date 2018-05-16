cmake_minimum_required (VERSION 3.11)
 
# Maps to Visual Studio solution file (BasicDeformationApp.sln)
# The solution will have only exe target.
# as Visual Studio projects (.vcproj)
set (CMAKE_USER_MAKE_RULES_OVERRIDE "${CMAKE_CURRENT_LIST_DIR}/CompilerOptions.cmake")
project (BasicDeformationApp)
 
# Collect sources into the variable SOURCE_FILES, HEADER_FILES without
# having to explicitly list each header and source file.
#
# CMake documentation states "We do not recommend using GLOB to collect a
# list of source files from your source tree. If no CMakeLists.txt file
# changes when a source is added or removed then the generated build system
# cannot know when to ask CMake to regenerate".
file (GLOB HEADER_FILES
      "src/*.h")
 
# Collect sources into the variable SOURCE_FILES
file (GLOB SOURCE_FILES
      "src/*.cpp")

# Create named folders for the sources within the .vcproj
# Empty name lists them directly under the .vcproj
# source_group("" FILES ${SOURCE_FILES})
source_group("Source Files" FILES ${SOURCE_FILES})
source_group("Headers" FILES ${HEADER_FILES})
 
# Set Properties->General->Configuration Type to Application(.exe)
# Creates app.exe with the listed sources (main.cpp)
# Adds sources to the Solution Explorer
add_executable (BasicDeformationApp ${SOURCE_FILES} ${HEADER_FILES})

# Properties->C/C++->General->Additional Include Directories
include_directories ("libigl/include"
					 "libigl/external/eigen"
					 "libigl/external/glad/include"
					 "libigl/external/glfw/include"
					 "libigl/external/imgui/."
					 "ImGuizmo/..")
target_link_libraries(BasicDeformationApp glfw3 glad opengl32 imgui)
# Turn on the ability to create folders to organize projects (.vcproj)
# It creates "CMakePredefinedTargets" folder by default and adds CMake
# defined projects like INSTALL.vcproj and ZERO_CHECK.vcproj
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
 
# Set compiler flags and options. 
# Here it is setting the Visual Studio warning level to 4
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
 
# Command to output information to the console
# Useful for displaying errors, warnings, and debugging
# message ("cxx Flags:" ${CMAKE_CXX_FLAGS})
 
# Sub-directories where more CMakeLists.txt exist
# add_subdirectory(src)
