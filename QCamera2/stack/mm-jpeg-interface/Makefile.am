ACLOCAL_AMFLAGS = -I m4

#libmmcamera2_sensor_modules.so
lib_LTLIBRARIES = libmmjpeg_interface.la

libmmjpeg_interface_la_CFLAGS = -DAMSS_VERSION=$(AMSS_VERSION)
libmmjpeg_interface_la_CFLAGS += -DMSM_CAMERA_GCC
libmmjpeg_interface_la_CFLAGS += -fPIC
libmmjpeg_interface_la_CFLAGS += -DUNIX_PATH_MAX=108
libmmjpeg_interface_la_CFLAGS += -DALOGD=fprintf
libmmjpeg_interface_la_CFLAGS += -Dstrlcpy=g_strlcpy
libmmjpeg_interface_la_CFLAGS += -DMM_JPEG_CONCURRENT_SESSIONS_COUNT=1


libmmjpeg_interface_la_CFLAGS += -I ${WORKSPACE}/hardware/qcom/camera/QCamera2/stack/mm-jpeg-interface/inc
libmmjpeg_interface_la_CFLAGS += -I ${WORKSPACE}/hardware/qcom/camera/QCamera2/stack/common
libmmjpeg_interface_la_CFLAGS += -I ${WORKSPACE}/hardware/libhardware/include
libmmjpeg_interface_la_CFLAGS += -I ${WORKSPACE}/system/core/include


libmmjpeg_interface_la_CFLAGS += -I ${WORKSPACE}/hardware/qcom/camera/QCamera2/stack/mm-camera-test/inc
libmmjpeg_interface_la_CFLAGS += -I ${WORKSPACE}/hardware/qcom/camera/QCamera2/stack/mm-camera-interface/inc
libmmjpeg_interface_la_CFLAGS += -I ${WORKSPACE}/hardware/qcom/camera/QCamera2/stack/common
libmmjpeg_interface_la_CFLAGS += -I ${WORKSPACE}/hardware/qcom/camera/mm-image-codec/qomx_core
libmmjpeg_interface_la_CFLAGS += -I ${WORKSPACE}/frameworks/native/include/media/openmax
libmmjpeg_interface_la_CFLAGS += -I ${WORKSPACE}/hardware/qcom/camera/mm-image-codec/qexif

libmmjpeg_interface_la_SOURCES = src/mm_jpeg_queue.c
libmmjpeg_interface_la_SOURCES += src/mm_jpeg_exif.c
libmmjpeg_interface_la_SOURCES += src/mm_jpeg.c
libmmjpeg_interface_la_SOURCES += src/mm_jpeg_interface.c
libmmjpeg_interface_la_SOURCES += src/mm_jpeg_ionbuf.c
libmmjpeg_interface_la_SOURCES += src/mm_jpegdec_interface.c
libmmjpeg_interface_la_SOURCES += src/mm_jpegdec.c

libmmjpeg_interface_la_LDFLAGS = -shared
libmmjpeg_interface_la_LIBADD = -ldl -lm

