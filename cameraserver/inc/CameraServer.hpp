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
#ifndef __CAMERA_SERVER_H__
#define __CAMERA_SERVER_H__
#include <vector>
#include <list>
#include <pthread.h>
#include "../libcamera/inc/camera.h"
#include "camera_parameters.h"
#include "CameraClientCommand.h"

using namespace camera;
struct ClientDescriptor {
    int mCameraId;
    int mClientFD;
    int mMaster;
    pthread_mutex_t mClientLock;
    pthread_t mThread;
    int mFrameFds[20];
    int mNumFrameFd;
} ;

class CameraObject : public ICameraListener {
public:
    CameraObject();                                   // Constructor of Camera Object
    virtual ~CameraObject();                          // Destructor of Camera Object

    /* listener methods */
    virtual void onError();
    virtual void onPreviewFrame(ICameraFrame* frame);
    virtual void onVideoFrame(ICameraFrame* frame);
    virtual void onPictureFrame(ICameraFrame* frame);
    virtual void onMetadataFrame(ICameraFrame *frame);
    pthread_mutex_t mCameraObjLock;
    ICameraDevice* mCamera;                                 // ICameraDevice instance
    CameraParams mParams;                                   // ICameraParameters instance used to hold camera parameters received from Camera
    int mOpenInstances;
};


class CameraServer
{
public:
    CameraServer();                          // The constructor will initialize class properties. It will open a socket and
    ~CameraServer();                         // The destructor will terminate socket thread and close the socket
    void start();
    bool delClientByFD(int fd);
    int  processCommand(ClientDescriptor &client,
            const ICameraCommandType &cmd, void* payload,
            ICameraCommandType &ack, void **ack_payload);
    int  dispatchFrame(int camId, ICameraCommand cmd, const ICameraFrame *frame);
private:
    int  openCamera(int camId, int &isMaster);
    int  closeCamera(int camId);
    int  startPreview(int camId);
    int  stopPreview(int camId);
    int  startRecording(int camId);
    int  stopRecording(int camId);
    int  getParameters(int camId);
    int  setParameters(int camId);
    int  takePicture(int camId);
    int  cancelPicture(int camId);
    int  enableFaceDetect(int camId, bool enable);
    int mNumberOfCameras;                          // Number of cameras on the device
    int mSocket;                                   // S
    int mCleanupPipeFd[2];
    std::vector<struct CameraInfo> mCameraInfo;    // Vector of camera info for every camera
    std::vector<CameraObject *> pCamObjects;        // Vector of camera objects allocated according number of cameras
    std::list<ClientDescriptor *> pClDescriptors; // Vector to keep data specific for every Client.
};

class SrvCameraObject: public CameraObject {
public:
    SrvCameraObject(CameraServer *srv, int camId);                                   // Constructor of Camera Object
    virtual ~SrvCameraObject();                          // Destructor of Camera Object
    virtual void onError();
    virtual void onPreviewFrame(ICameraFrame* frame);
    virtual void onVideoFrame(ICameraFrame* frame);
    virtual void onPictureFrame(ICameraFrame* frame);
    virtual void onMetadataFrame(ICameraFrame *frame);
private:
    CameraServer *mServer;
    int mCamId;
};

#endif // __CAMERA_SERVER_H__


