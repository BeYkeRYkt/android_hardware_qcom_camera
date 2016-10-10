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
#ifndef __CAMERA_CLIENT_HPP__
#define __CAMERA_CLIENT_HPP__
#include "camera.h"
#include "pthread.h"
#include "camera_parameters.h"
#include "camera_memory.h"

#define SEVER_SOCKET_PATH  "/root/cam_srv_sock"

using namespace camera;

typedef struct _CameraFuncType {
    int camera_func[4];
} CameraFuncType;

class ICameraClientFrame : public ICameraFrame
{
private:

public:
    ICameraClientFrame( int64_t timestamp,
            QCamera2Frame *frame, int index, int SocketFD);
    virtual ~ICameraClientFrame();

    virtual uint32_t acquireRef();

    virtual uint32_t releaseRef();

    void setSocketFD(int mSocketFD);
private:
    uint32_t refs_;
    uint32_t index_;
    int mSocketFD;
};

class ICameraClientParams : public CameraParams
{
public:
    ICameraClientParams();
    ~ICameraClientParams();
    virtual int init(ICameraDevice* device = NULL);
    virtual int commit();
    void InitComm(int socketFD, pthread_mutex_t *mAPIlock);
    void SetSlaveDevice(bool SlaveDevice);

private:
    int mSocketFD;
    pthread_mutex_t *mAPIlock;
    bool mSlaveDevice;

public:
    pthread_cond_t mAPIcond;

};


class ICameraClient
{

public:
    /* Constructs ICameraClient object opens a socket to the server and starts listen therad. */
    ICameraClient();

    /* Stops listen therad, close the socket to the server, destroy ICameraClient object */
    ~ICameraClient();

    /* Initialize API. It should be called first after creating CameraClient object */

    int Init();

    int OpenCamera(int camID);
    int CloseCamera(int camID);

    int StartPreview();
    int StopPreview();

    int StartRecording();
    int StopRecording();

    int TakePicture();
    int CancelPicture();

    void AddListener(ICameraListener* listener);
    void RemoveListener(ICameraListener* listener);

    int setParameters();
    int getParameters();

    int startAutoFocus() {};
    int stopAutoFocus()  {};

    int sendFaceDetectCommand(bool turn_on);

    void SetNumCameras(int NumCameras);
    int  GetNumCameras();

    void SetCameraInfo(CameraFuncType *CameraFunc);
    void GetCameraInfo( int idx, struct CameraInfo  *info);

    int GetIsActive() { return mIsActive;}

    void SetIsActive(int isActive) { mIsActive = isActive;}

    /* Deinitialize API. It should be called before client object is deleted */
    int deInit();

private:
    void ThreadLoop();
    pthread_t  mThread_id;
    bool mPreviewRunnnig;
    bool mVideoRunning;
    bool mPictureStarted;
    pthread_mutex_t mAPIlock;
    CameraFuncType mCameraFunc;
    int mNumCameras;
    int mIsActive;

public:
    std::vector <CameraMemory *> pMem;
    int mSocketFD;
    ICameraClientParams params;
    pthread_cond_t mAPIcond;
    int  mError;
    ICameraListener *mCamListner;

};
#endif //__CAMERA_CLIENT_HPP__

