INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_TOA toa)

FIND_PATH(
    TOA_INCLUDE_DIRS
    NAMES toa/api.h
    HINTS $ENV{TOA_DIR}/include
        ${PC_TOA_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    TOA_LIBRARIES
    NAMES gnuradio-toa
    HINTS $ENV{TOA_DIR}/lib
        ${PC_TOA_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(TOA DEFAULT_MSG TOA_LIBRARIES TOA_INCLUDE_DIRS)
MARK_AS_ADVANCED(TOA_LIBRARIES TOA_INCLUDE_DIRS)

