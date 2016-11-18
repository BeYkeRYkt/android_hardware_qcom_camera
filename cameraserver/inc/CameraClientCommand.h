/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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
 *
 */
#ifndef __CAMERA_CLIENT_COMMAND_HPP__
#define __CAMERA_CLIENT_COMMAND_HPP__

#include "camera.h"

namespace camera
{

enum ICameraCommand {
    GET_NUM_CAMERAS = 1,      // 1
    GET_NUM_CAMERAS_DONE,     // 2
    GET_CAMERAS_INFO,         // 3
    GET_CAMERAS_INFO_DONE,    // 4
    OPEN_CAMERA,              // 5
    OPEN_CAMERA_DONE,         // 6
    CLOSE_CAMERA,             // 7
    CLOSE_CAMERA_DONE,        // 8
    GET_PARAMETERS,           // 9
    GET_PARAMETERS_DONE,      // 10
    SET_PARAMETERS,           // 11
    SET_PARAMETERS_DONE,      // 12
    ENABLE_FACE_DETECT,       // 13
    ENABLE_FACE_DETECT_DONE,  // 14
    START_PREVIEW,            // 15
    START_PREVIEW_DONE,       // 16
    STOP_PREVIEW,             // 17
    STOP_PREVIEW_DONE,        // 18
    START_RECORDING,          // 19
    START_RECORDING_DONE,     // 20
    STOP_RECORDING,           // 21
    STOP_RECORDING_DONE,      // 22
    TAKE_PICTURE,             // 23
    TAKE_PICTURE_DONE,        // 24
    CANCEL_PICTURE,           // 25
    CANCEL_PICTURE_DONE,      // 26
    NEW_PREVIEW_FRAME,        // 27
    NEW_VIDEO_FRAME,          // 28
    NEW_SNAPSHOT_FRAME,       // 29
    NEW_META_FRAME,           // 30
    RELEASE_FRAME,            // 31
    RELEASE_FRAME_DONE,       // 32
    STOP_SESSION,             // 33
    STOP_SESSION_DONE,        // 34
    SERVER_NOTIFICATION,      // 35
};

typedef struct _ICameraCommandFrameType{
    int index;
    int bufSize;
    uint64_t timestamp;
    FaceRoi faceROI;
} ICameraCommandFrameType;
    
typedef struct _ICameraCommandType{
    int type;
    int payload_size;
} ICameraCommandType;

}
#endif //__CAMERA_CLIENT_COMMAND_HPP__

