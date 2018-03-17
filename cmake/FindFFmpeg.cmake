# Finds the FFmpeg library
#
# This will define the following variables
#    FFmpeg_FOUND
#    FFmpeg_INCLUDE_DIRS
#    FFmpeg_LIBRARIES
#
# and the following imported targets
#    FFmpeg::avcodec
#    FFmpeg::avdevice
#    FFmpeg::avfilter
#    FFmpeg::avformat
#    FFmpeg::avresample
#    FFmpeg::avutil
#    FFmpeg::postproc
#    FFmpeg::swresample
#    FFmpeg::swscale

find_package(PkgConfig)
pkg_check_modules(PC_avcodec    QUIET libavcodec)
pkg_check_modules(PC_avdevice   QUIET libavdevice)
pkg_check_modules(PC_avfilter   QUIET libavfilter)
pkg_check_modules(PC_avformat   QUIET libavformat)
pkg_check_modules(PC_avresample QUIET libavresample)
pkg_check_modules(PC_avutil     QUIET libavutil)
pkg_check_modules(PC_postproc   QUIET libpostproc)
pkg_check_modules(PC_swresample QUIET libswresample)
pkg_check_modules(PC_swscale    QUIET libswscale)

find_path(avcodec_INCLUDE_DIR libavcodec/avcodec.h PATHS ${PC_avcodec_INCLUDE_DIRS})
find_path(avdevice_INCLUDE_DIR libavdevice/avdevice.h PATHS ${PC_avdevice_INCLUDE_DIRS})
find_path(avfilter_INCLUDE_DIR libavfilter/avfilter.h PATHS ${PC_avfilter_INCLUDE_DIRS})
find_path(avformat_INCLUDE_DIR libavformat/avformat.h PATHS ${PC_avformat_INCLUDE_DIRS})
find_path(avresample_INCLUDE_DIR libavresample/avresample.h PATHS ${PC_avresample_INCLUDE_DIRS})
find_path(avutil_INCLUDE_DIR libavutil/avutil.h PATHS ${PC_avutil_INCLUDE_DIRS})
find_path(postproc_INCLUDE_DIR libpostproc/postprocess.h PATHS ${PC_postproc_INCLUDE_DIRS})
find_path(swresample_INCLUDE_DIR libswresample/swresample.h PATHS ${PC_swresample_INCLUDE_DIRS})
find_path(swscale_INCLUDE_DIR libswscale/swscale.h PATHS ${PC_swscale_INCLUDE_DIRS})

find_library(avcodec_LIBRARY avcodec PATHS ${PC_avcodec_LIBDIR} ${PC_avcodec_LIBRARY_DIRS})
find_library(avdevice_LIBRARY avdevice PATHS ${PC_avdevice_LIBDIR} ${PC_avdevice_LIBRARY_DIRS})
find_library(avfilter_LIBRARY avfilter PATHS ${PC_avfilter_LIBDIR} ${PC_avfilter_LIBRARY_DIRS})
find_library(avformat_LIBRARY avformat PATHS ${PC_avformat_LIBDIR} ${PC_avformat_LIBRARY_DIRS})
find_library(avresample_LIBRARY avresample PATHS ${PC_avresample_LIBDIR} ${PC_avresample_LIBRARY_DIRS})
find_library(avutil_LIBRARY avutil PATHS ${PC_avutil_LIBDIR} ${PC_avutil_LIBRARY_DIRS})
find_library(postproc_LIBRARY postproc PATHS ${PC_postproc_LIBDIR} ${PC_postproc_LIBRARY_DIRS})
find_library(swresample_LIBRARY swresample PATHS ${PC_swresample_LIBDIR} ${PC_swresample_LIBRARY_DIRS})
find_library(swscale_LIBRARY swscale PATHS ${PC_swscale_LIBDIR} ${PC_swscale_LIBRARY_DIRS})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    FFmpeg
    REQUIRED_VARS
    avcodec_INCLUDE_DIR
    avdevice_INCLUDE_DIR
    avfilter_INCLUDE_DIR
    avformat_INCLUDE_DIR
    avresample_INCLUDE_DIR
    avutil_INCLUDE_DIR
    postproc_INCLUDE_DIR
    swresample_INCLUDE_DIR
    swscale_INCLUDE_DIR
    avdevice_LIBRARY
    avfilter_LIBRARY
    avformat_LIBRARY
    avresample_LIBRARY
    avutil_LIBRARY
    postproc_LIBRARY
    swresample_LIBRARY
    swscale_LIBRARY)

mark_as_advanced(
    avcodec_INCLUDE_DIR
    avdevice_INCLUDE_DIR
    avfilter_INCLUDE_DIR
    avformat_INCLUDE_DIR
    avresample_INCLUDE_DIR
    avutil_INCLUDE_DIR
    postproc_INCLUDE_DIR
    swresample_INCLUDE_DIR
    swscale_INCLUDE_DIR
    avdevice_LIBRARY
    avfilter_LIBRARY
    avformat_LIBRARY
    avresample_LIBRARY
    avutil_LIBRARY
    postproc_LIBRARY
    swresample_LIBRARY
    swscale_LIBRARY)

if(FFmpeg_FOUND)
    set(FFmpeg_INCLUDE_DIRS
        ${avcodec_INCLUDE_DIR}
        ${avdevice_INCLUDE_DIR}
        ${avfilter_INCLUDE_DIR}
        ${avformat_INCLUDE_DIR}
        ${avresample_INCLUDE_DIR}
        ${avutil_INCLUDE_DIR}
        ${postproc_INCLUDE_DIR}
        ${swresample_INCLUDE_DIR}
        ${swscale_INCLUDE_DIR})
    set(FFmpeg_LIBRARIES
        ${avcodec_LIBRARY}
        ${avdevice_LIBRARY}
        ${avfilter_LIBRARY}
        ${avformat_LIBRARY}
        ${avresample_LIBRARY}
        ${avutil_LIBRARY}
        ${postproc_LIBRARY}
        ${swresample_LIBRARY}
        ${swscale_LIBRARY})

    if(NOT TARGET FFmpeg::avcodec)
        add_library(FFmpeg::avcodec INTERFACE IMPORTED)
        set_target_properties(
            FFmpeg::avcodec
            PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${avcodec_INCLUDE_DIR}
            INTERFACE_LINK_LIBRARIES ${avcodec_LIBRARY})
    endif()
    if(NOT TARGET FFmpeg::avdevice)
        add_library(FFmpeg::avdevice INTERFACE IMPORTED)
        set_target_properties(
            FFmpeg::avdevice
            PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${avdevice_INCLUDE_DIR}
            INTERFACE_LINK_LIBRARIES ${avdevice_LIBRARY})
    endif()
    if(NOT TARGET FFmpeg::avfilter)
        add_library(FFmpeg::avfilter INTERFACE IMPORTED)
        set_target_properties(
            FFmpeg::avfilter
            PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${avfilter_INCLUDE_DIR}
            INTERFACE_LINK_LIBRARIES ${avfilter_LIBRARY})
    endif()
    if(NOT TARGET FFmpeg::avformat)
        add_library(FFmpeg::avformat INTERFACE IMPORTED)
        set_target_properties(
            FFmpeg::avformat
            PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${avformat_INCLUDE_DIR}
            INTERFACE_LINK_LIBRARIES ${avformat_LIBRARY})
    endif()
    if(NOT TARGET FFmpeg::avresample)
        add_library(FFmpeg::avresample INTERFACE IMPORTED)
        set_target_properties(
            FFmpeg::avresample
            PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${avresample_INCLUDE_DIR}
            INTERFACE_LINK_LIBRARIES ${avresample_LIBRARY})
    endif()
    if(NOT TARGET FFmpeg::avutil)
        add_library(FFmpeg::avutil INTERFACE IMPORTED)
        set_target_properties(
            FFmpeg::avutil
            PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${avutil_INCLUDE_DIR}
            INTERFACE_LINK_LIBRARIES ${avutil_LIBRARY})
    endif()
    if(NOT TARGET FFmpeg::postproc)
        add_library(FFmpeg::postproc INTERFACE IMPORTED)
        set_target_properties(
            FFmpeg::postproc
            PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${postproc_INCLUDE_DIR}
            INTERFACE_LINK_LIBRARIES ${postproc_LIBRARY})
    endif()
    if(NOT TARGET FFmpeg::swresample)
        add_library(FFmpeg::swresample INTERFACE IMPORTED)
        set_target_properties(
            FFmpeg::swresample
            PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${swresample_INCLUDE_DIR}
            INTERFACE_LINK_LIBRARIES ${swresample_LIBRARY})
    endif()
    if(NOT TARGET FFmpeg::swscale)
        add_library(FFmpeg::swscale INTERFACE IMPORTED)
        set_target_properties(
            FFmpeg::swscale
            PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${swscale_INCLUDE_DIR}
            INTERFACE_LINK_LIBRARIES ${swscale_LIBRARY})
    endif()
endif()
