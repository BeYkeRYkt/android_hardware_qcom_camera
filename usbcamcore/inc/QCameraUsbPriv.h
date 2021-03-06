/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ANDROID_HARDWARE_QCAMERA_USB_PRIV_H
#define ANDROID_HARDWARE_QCAMERA_USB_PRIV_H

#include <linux/videodev2.h>
#include <QCameraParameters.h>
#include <media/hardware/HardwareAPI.h>

using namespace qcamera;
namespace android {

/* File name length in number of characters */
#define FILENAME_LENGTH     (256)

/* Number of display buffers (in addition to minimum number of undequed buffers */
#define PRVW_DISP_BUF_CNT   2

/* Number of V4L2 capture  buffers. */
#define PRVW_CAP_BUF_CNT    4

/* Maximum buffer size for JPEG output in number of bytes */
#define MAX_JPEG_BUFFER_SIZE    (1024 * 1024)

/* Preview loop commands */
#define USB_CAM_PREVIEW_EXIT    (0x100)
#define USB_CAM_PREVIEW_PAUSE   (0x101)
#define USB_CAM_PREVIEW_TAKEPIC (0x200)

/******************************************************************************
 * Macro function to input validate device handle
 *****************************************************************************/
#define VALIDATE_DEVICE_HDL(camHal, device, ret_err_code)     {\
    if(device && device->priv){\
        camHal = (camera_hardware_t *)device->priv;\
    }else{\
        ALOGE("%s: Null device or device->priv", __func__);\
        return ret_err_code;\
    }\
}\

/******************************************************************************
 * Macro function to check return status of a function, log and exit the thread
 *****************************************************************************/
#define ERROR_CHECK_EXIT_THREAD(rc, string)    {\
    if(rc < 0) {\
        ALOGE("%s: Error %s", __func__, string);\
        return (void *)-1;\
    }\
}

/******************************************************************************
 * Macro function to check return status of a function, log and exit
 *****************************************************************************/
#define ERROR_CHECK_EXIT(rc, string)    {\
    if(rc < 0) {\
        ALOGE("%s: Error %s", __func__, string);\
        return -1;\
    }\
}

/******************************************************************************
* Macro function to Print the parameter string 1000 characters at a time
******************************************************************************/
#define PRINT_PARAM_STR(parms)    {\
        char temp[1001] = {0};\
        int n=0;\
        while(1) {\
            strlcpy(temp,parms+n,1000);\
            ALOGD("parms = %s", temp);\
            if (strlen(temp) < 1000) break;\
            n += 1000;\
        }\
    }\

/******************************************************************************
 * Macro function to open camera
 *****************************************************************************/
#define USB_CAM_OPEN(camHal)    {\
        camHal->fd = open(camHal->dev_name, O_RDWR | O_NONBLOCK, 0);\
        if(!camHal->fd)\
            ALOGE("%s: Error in open", __func__);\
        else\
            ALOGD("%s: Successfully opened", __func__);\
        }\

/******************************************************************************
 * Macro function to close camera
 *****************************************************************************/
#define USB_CAM_CLOSE(camHal) {\
        int rc;\
        if(camHal->fd){\
            rc = close(camHal->fd);\
            if(0 > rc){\
                ALOGE("%s: close failed ", __func__);\
            }\
            else{\
                camHal->fd = 0;\
                ALOGD("%s: close successful", __func__);\
            }\
        }\
    }\

struct bufObj {
    void    *data;
    int     len;
};

#define MAX_BUFFER_COUNT 16

typedef struct {
	int size;
	int fd;
	int main_ion_fd;
	int handle;
}QCameraHalMemInfo_t;

typedef struct {
	int size;
	int fd;
	int offset;
}private_buffer_handle_t;


typedef struct {
	int buffer_count;
	int stride[MAX_BUFFER_COUNT];
	int local_flag[MAX_BUFFER_COUNT];
	buffer_handle_t *buffer_handle[MAX_BUFFER_COUNT];
	QCameraHalMemInfo_t mem_info[MAX_BUFFER_COUNT];
	camera_memory_t *camera_memory[MAX_BUFFER_COUNT];
	struct private_handle_t *private_buffer_handle[MAX_BUFFER_COUNT];
}QCameraHalMemory_t;

typedef enum {
  JPEG_JOB_STATUS_DONE = 0,
  JPEG_JOB_STATUS_ERROR
} jpeg_job_status_t;


typedef struct {
    camera_device                       hw_dev;
    Mutex                               lock;
    int                                 previewEnabledFlag;
    int                                 videorecordingEnableFlag;
    int                                 prvwStoppedForPicture;
    int                                 msgEnabledFlag;
    int                                 snapshotEnabledFlag;
    int                                 preview_thread_enable;
    volatile int                        prvwCmdPending;
    volatile int                        prvwCmd;
    pthread_t                           previewThread;
    pthread_t                           recordingThread;
    pthread_t                           takePictureThread;

    camera_notify_callback              notify_cb;
    camera_data_callback                data_cb;
    camera_data_timestamp_callback      data_cb_timestamp;
    camera_request_memory               get_memory;
    Vector<camera_memory_t*>            prevFreeBufs;
    Vector<camera_memory_t*>            pictFreeBufs;
    Vector<camera_memory_t*>            videoFreeBufs;
    void*                               cb_ctxt;

    /* capture related members */
    /* prevFormat is pixel format of preview buffers that are exported */
    int                                 prevFormat;
    int                                 prevFps;
    int                                 prevWidth;
    int                                 prevHeight;

    int                                 videoWidth;
    int                                 videoHeight;
    QCameraHalMemory_t                  videoMem;
    /* captureFormat is internal setting for USB camera buffers */
    int                                 recordingFormat;
    int                                 previewFormat;
    int                                 captureFormat;
    char                                dev_name[FILENAME_LENGTH];
    int                                 fd;
    char                                h264_dev_name[FILENAME_LENGTH];
    int                                 h264_fd;
    unsigned int                        n_buffers;
    unsigned int                        n_h264_buffers;
    struct v4l2_buffer                  curCaptureBuf;
    struct v4l2_buffer                  curVideoBuf;
    struct bufObj                       *buffers;
    struct bufObj                       *h264Buffers;

    /* Display related members */
    preview_stream_ops*                 window;
    QCameraHalMemory_t                  previewMem;
    /* dispFormat is preview display format.Same as preview buffer format*/
    int                                 dispFormat;
    int                                 dispWidth;
    int                                 dispHeight;

    /* MJPEG decoder related members */
    /* MJPEG decoder object */
    void*                               mjpegd;

    /* JPEG picture and thumbnail related members */
    int                                 pictFormat;
    int                                 pictWidth;
    int                                 pictHeight;
    int                                 pictJpegQlty;
    Vector<Size>                        prevSizes;
    Vector<Size>                        pictSizes;
    Vector<Size>                        videoSizes;
    int                                 thumbnailWidth;
    int                                 thumbnailHeight;
    int                                 thumbnailJpegQlty;
    QCameraHalMemory_t                  pictMem;
    int                                 takePictInProgress;
    int                                 jpegEncInProgress;
    pthread_mutex_t                     jpegEncMutex;
    pthread_cond_t                      jpegEncCond;

    /* */
    QCameraParameters                   qCamParams;
    String8                             prevSizeValue;
    String8                             prevSizeValues;
    String8                             pictSizeValue;
    String8                             pictSizeValues;
    String8                             thumbnailSizeValues;
    String8                             vidSizeValue;
    String8                             vidSizeValues;
    String8                             pictFormatValues;
    String8                             prevFormatValues;
    String8                             prevFpsRangesValues;

    camera_memory_t *mMetadata[MM_CAMERA_MAX_NUM_FRAMES];
    native_handle_t *mNativeHandle[MM_CAMERA_MAX_NUM_FRAMES];

    camera_memory_t* getFreePrevBuffer(){
        if(prevFreeBufs.size() == 0)
            return 0;
        camera_memory_t* ret = prevFreeBufs.editTop();
        prevFreeBufs.pop();
        return ret;
    }

    void returnFreePrevBuffer(camera_memory_t* buf){
        prevFreeBufs.push_front(buf);
    }

    camera_memory_t* getFreeVideoBuffer(){
        if(videoFreeBufs.size() == 0)
            return 0;
        camera_memory_t* ret = videoFreeBufs.editTop();
        videoFreeBufs.pop();
        return ret;
    }

    void returnFreeVideoBuffer(camera_memory_t* buf){
        videoFreeBufs.push_front(buf);
    }

} camera_hardware_t;


}; // namespace android

#endif /* ANDROID_HARDWARE_QCAMERA_USB_PRIV_H */
