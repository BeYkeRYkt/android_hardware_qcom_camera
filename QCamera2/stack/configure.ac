# configure.ac -- Autoconf script for gps loc_api
#
# Process this file with autoconf to produce a configure script

# Requires autoconf tool later than 2.61
AC_PREREQ(2.61)
# Initialize the gps loc_api package version 1.0.0
AC_INIT([camera-hal],1.0.0)

AM_INIT_AUTOMAKE([-Werror -Wall gnu foreign subdir-objects])

# Disables auto rebuilding of configure, Makefile.ins
AM_MAINTAINER_MODE
# defines some macros variable to be included by source
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_PROG_LIBTOOL
AC_PROG_CXX
AM_PROG_AR
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_AWK
AC_PROG_CPP
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET
PKG_PROG_PKG_CONFIG

# Checks for libraries.


AC_SUBST([CPPFLAGS])

AC_ARG_WITH([glib],
      AC_HELP_STRING([--with-glib],
         [enable glib, building HLOS systems which use glib]))

AM_CONDITIONAL(USE_GLIB, test "x${with_glib}" = "xyes")

AC_CONFIG_FILES([ \
        Makefile \
        mm-jpeg-interface/Makefile \
        mm-camera-interface/Makefile \
        mm-camera-test/Makefile \
        ])
AC_OUTPUT
