cmake_minimum_required(VERSION 3.22)
project(turtlelib)

# Find your dependencies.
# Many libraries ship with files that allow CMake to find them
# Then general behavior is to call "find_package" but the options
# provided are package specific.  Usually there is then a CMAKE variable
# That is defined to reference the library
# here: we find the eigen library as per the instruction
# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
# find_package(Eigen3 3.3 REQUIRED)
find_package(Armadillo REQUIRED)
# Create a library.  Can specify if it is shared or static but usually
# you don't need or want to.
# name is the name of the library without the extension or lib prefix
# name creates a cmake "target"
add_library(turtlelib src/geometry2d.cpp 
                      src/se2d.cpp 
                      src/svg.cpp
                      src/diff_drive.cpp
                      src/ekf_slam.cpp
                      src/landmark_calc.cpp
                      )

# Use target_include_directories so that #include"mylibrary/header.hpp" works
# The use of the <BUILD_INTERFACE> and <INSTALL_INTERFACE> is because when
# Using the library from the build directory or after installation
# During build, the headers are read from the source code directory
# When used from the installed location, headers are in the
# system include/ directory
target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

# specify additional compilation flags for the library
# Public causes the flags to propagate to anything
# that links against this library
target_compile_options(turtlelib PUBLIC -Wall -Wextra -pedantic)

# Enable c++17 support.
# Public causes the features to propagate to anything
# that links against this library
target_compile_features(turtlelib PUBLIC cxx_std_17)

# Create an executable from the following source code files
# The Name of the executable creates a cmake "target"

#TODO: uncomment the following line and replace Name with the name of your executable
add_executable(frame_main src/frame_main.cpp)
configure_file(${CMAKE_SOURCE_DIR}/exercises/B6_frame_input.txt frame_input.txt COPYONLY)
target_link_libraries(frame_main turtlelib)

# Use target_link_libraries to add dependencies to a "target"
# (e.g., a library or executable)
# This will automatically add all required library files
# that need to be linked
# and paths to th locations of header files
# target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# install the include files by copying the whole include directory
install(DIRECTORY include/turtlelib DESTINATION include)

# Create a CMake Exported Target containing the lib and exe.
# Also create CMake Export called projet_name-targets
# The CMake Export contains files that allow other CMake projects
# to find this project. It must be installed separately.
install(TARGETS ${PROJECT_NAME} EXPORT turtlelib-targets)

# The project_name-targets created by install(TARGETS) needs to be installed.
# install(EXPORT ...) will generate a file called project_name-config.cmake
# that contains the exported targets.
# After installation this file will then be found when calling
# find_package(project_name) from another cmake project
# A user can then target_link_libraries(target project_name::library)
# to use the libraries installed here
install(EXPORT turtlelib-targets
        FILE turtlelib-config.cmake
        NAMESPACE turtlelib::
        DESTINATION lib/cmake/${PROJECT_NAME})


# # first we can indicate the documentation build as an option and set it to ON by default
# option(BUILD_DOC "Build documentation" ON)

find_package(Doxygen)

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" ON)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

if( NOT CMAKE_CROSSCOMPILING)
    include(CTest)

    # CTest sets BUILD_TESTING to on. To disable tests add -DBUILD_TESTING=OFF when invoking cmake
    if(BUILD_TESTING)
        # Find the Unit testing framework. In this example, Catch2
        find_package(Catch2 3 REQUIRED)
        find_package(Armadillo REQUIRED)
        find_package(LAPACK REQUIRED)
        find_package(BLAS REQUIRED)
        include_directories(include ${ARMADILLO_INCLUDE_DIRS})
        # add_definitions(-DARMA_DONT_USE_WRAPPER)
        # A test is just an executable that is linked against the unit testing library
        add_executable(my_test_exe tests/test_geometry2d.cpp 
                                tests/test_se2d.cpp 
                                tests/test_svg.cpp
                                tests/test_diff_drive.cpp
                                )
        add_executable(test_arma_lib tests/test_ekf_slam.cpp
                                    tests/test_landmark_calc.cpp)
        target_link_libraries(my_test_exe Catch2::Catch2WithMain turtlelib)
        target_link_libraries(test_arma_lib PRIVATE Catch2::Catch2WithMain turtlelib ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES})

        target_include_directories(my_test_exe
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:include/>)

        target_include_directories(test_arma_lib PUBLIC ${ARMADILLO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/)

        # register the test with CTest, telling it what executable to run
        add_test(NAME the_test_name COMMAND my_test_exe)
        add_test(NAME test_arma_test COMMAND test_arma_lib)
    endif()
endif()