# Finds the png++ library
#
# This will define the following variables
#    png++_FOUND
#    png++_INCLUDE_DIRS
#    png++_LIBRARIES
#
# and the following imported targets
#     png++::png++

find_package(PkgConfig)
pkg_check_modules(PC_png++ QUIET png++)

find_path(
    png++_INCLUDE_DIR
    png++/png.hpp
    PATHS ${PC_png++_INCLUDE_DIRS}
    PATH_SUFFIXES png++)

find_package(PNG REQUIRED)

mark_as_advanced(png++_INCLUDE_DIRS)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(png++ REQUIRED_VARS png++_INCLUDE_DIR PNG_FOUND)

if(png++_FOUND)
    set(png++_INCLUDE_DIRS ${PNG_INCLUDE_DIRS} ${png++_INCLUDE_DIR})
    set(png++_LIBRARIES ${PNG_LIBRARIES})
endif()

if(png++_FOUND AND NOT TARGET png++::png++)
    add_library(png++::png++ INTERFACE IMPORTED)
    set_target_properties(
        png++::png++
        PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES ${png++_INCLUDE_DIR}
        INTERFACE_LINK_LIBRARIES PNG::PNG)
endif()
