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

enum ICameraCommand {
    GET_NUM_CAMERAS = 1,
    GET_NUM_CAMERAS_DONE,
    GET_CAMERAS_INFO,
    GET_CAMERAS_INFO_DONE,
    OPEN_CAMERA,
    OPEN_CAMERA_DONE,
    CLOSE_CAMERA,
    CLOSE_CAMERA_DONE,
    GET_PARAMETERS,
    GET_PARAMETERS_DONE,
    SET_PARAMETERS,
    SET_PARAMETERS_DONE,
    ENABLE_FACE_DETECT,
    ENABLE_FACE_DETECT_DONE,
    START_PREVIEW,
    START_PREVIEW_DONE,
    STOP_PREVIEW,
    STOP_PREVIEW_DONE,
    START_RECORDING,
    START_RECORDING_DONE,
    STOP_RECORDING,
    STOP_PRECORDING_DONE,
    TAKE_PICTURE,
    TAKE_PICTURE_DONE,
    CANCEL_PICTURE,
    CANCEL_PICTURE_DONE,
    NEW_PREVIEW_FRAME,
    NEW_VIDEO_FRAME,
    NEW_SNAPSHOT_FRAME,

    NEW_META_FRAME,
    RELEASE_FRAME,
    STOP_SESSION,
    STOP_SESSION_DONE,
};

typedef struct _ICameraCommandFrameType{
    int index;
    int bufSize;
    uint64_t timestamp;
} ICameraCommandFrameType;
    
typedef struct _ICameraCommandType{
    int type;
    int payload_size;
} ICameraCommandType;
#endif //__CAMERA_CLIENT_COMMAND_HPP__

