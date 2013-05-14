# - Try to find Portaudio
# Once done this will define
#
#  PORTAUDIO_FOUND - system has Portaudio
#  PORTAUDIO_INCLUDE_DIRS - the Portaudio include directory
#  PORTAUDIO_LIBRARIES - Link these to use Portaudio

include(FindPkgConfig)
if(CMAKE_SYSTEM_NAME STREQUAL "FreeBSD")
  message( "--- Using Portaudio2 for FreeBSD")
  set(PORTAUDIO_FOUND TRUE)
  set(PORTAUDIO_INCLUDE_DIRS "/usr/local/include/portaudio2")
  set(PORTAUDIO_LIBRARIES "/usr/local/lib/portaudio2/libportaudio.so")
else()
  pkg_check_modules(PC_PORTAUDIO portaudio)

find_path(PORTAUDIO_INCLUDE_DIRS
  NAMES
    portaudio.h
  PATHS
      /usr/local/include
      /usr/include
  HINTS
    ${PC_PORTAUDIO_INCLUDE_DIR}
)

find_library(PORTAUDIO_LIBRARIES
  NAMES
    portaudio
  PATHS
      /usr/local/lib
      /usr/lib
      /usr/lib64
  HINTS
    ${PC_PORTAUDIO_LIBDIR}
)
endif()
mark_as_advanced(PORTAUDIO_INCLUDE_DIRS PORTAUDIO_LIBRARIES)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PORTAUDIO DEFAULT_MSG PORTAUDIO_INCLUDE_DIRS PORTAUDIO_LIBRARIES)
