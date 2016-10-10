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
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include "CameraClient.hpp"
#include "CameraClientCommand.h"
#include "CameraClientUtil.hpp"


static void *MainThreadLoop(void *arg)
{
    ICameraClient *pCameraClient = (ICameraClient *)arg;
    ICameraCommandType CameraCommand;
    uint8_t *commandPayload = NULL;
    int readBytes = 0;
    int recvFD = 0;

    if (pCameraClient->mSocketFD > 0)
    {
      while (pCameraClient->GetIsActive())
      {

        readBytes = socket_recvmsg(pCameraClient->mSocketFD, &CameraCommand, sizeof(ICameraCommandType), &recvFD);
        if ((CameraCommand.payload_size != 0)/* && (readBytes == sizeof(ICameraCommandType))*/)
        {
            commandPayload = (uint8_t *)malloc(CameraCommand.payload_size);
            if (commandPayload == NULL)
            {
                fprintf(stderr,"Fail to allocate memory for command payload\n");
                break;
            }
            memset(commandPayload, 0, CameraCommand.payload_size);
            readBytes = socket_recvmsg(pCameraClient->mSocketFD, commandPayload, CameraCommand.payload_size, &recvFD);
        } else {
            readBytes = 0;
        }
        /*if (readBytes && readBytes == CameraCommand.payload_size)*/
        {
            switch (CameraCommand.type)
            {
                case GET_PARAMETERS_DONE:
                {
                    char *paramsBuf = (char *)commandPayload;
                    if (paramsBuf != NULL) {
                        pCameraClient->params.readObject(paramsBuf);
                    } else {
                        fprintf(stderr,"Error receiving camera parameters\n");
                    }
                    pthread_cond_signal(&pCameraClient->params.mAPIcond);
                    break;
                }
                case GET_NUM_CAMERAS_DONE:
                {
                    pCameraClient->SetNumCameras(*(int *)commandPayload);
                    pthread_cond_signal(&pCameraClient->mAPIcond);
                    break;
                }
                case GET_CAMERAS_INFO_DONE:
                    pCameraClient->SetCameraInfo((CameraFuncType *)commandPayload);
                    pthread_cond_signal(&pCameraClient->mAPIcond);
                    break;
                case SET_PARAMETERS_DONE:
                    pthread_cond_signal(&pCameraClient->params.mAPIcond);
                    break;
                case STOP_PREVIEW_DONE:
                        fprintf(stderr,"STOP_PREVIEW_DONE\n");
                case OPEN_CAMERA_DONE:
                case ENABLE_FACE_DETECT_DONE:
                case START_PREVIEW_DONE:
                case START_RECORDING_DONE:
                case STOP_PRECORDING_DONE:
                case TAKE_PICTURE_DONE:
                case CANCEL_PICTURE_DONE:
                    /*pCameraClient->mError = *(int *)commandPayload;*/
                    pthread_cond_signal(&pCameraClient->mAPIcond);
                    break;
                case NEW_PREVIEW_FRAME:
                {
                    CameraMemory *pnewMemEntry;
                    ICameraClientFrame *pPreviewFrame;
                    ICameraCommandFrameType *pnewFrame =
                     (ICameraCommandFrameType *)commandPayload;
                    if (recvFD)
                    {
                        pnewMemEntry = new CameraMemory(recvFD, pnewFrame->bufSize);
                        pCameraClient->pMem.push_back(pnewMemEntry);
                    }

                    pPreviewFrame = new ICameraClientFrame(0,
                            &pCameraClient->pMem[pnewFrame->index]->frame,
                            pnewFrame->index,
                            pCameraClient->mSocketFD);
                    pPreviewFrame->acquireRef();
                    pCameraClient->mCamListner->onPreviewFrame(static_cast<ICameraFrame*>(pPreviewFrame));

                    break;
                }
                case STOP_SESSION_DONE:
                    pCameraClient->SetIsActive(0);
                    pthread_cond_signal(&pCameraClient->mAPIcond);
                    break;
                default:
                    break;
            }
        }
        if (commandPayload) {
            free(commandPayload);
            commandPayload = NULL;
        }
        memset(&CameraCommand, 0, sizeof(ICameraCommandType));
      }
    }
}

ICameraClientFrame::ICameraClientFrame( int64_t timestamp,
            QCamera2Frame *frame, int index, int SocketFD) :
            refs_(0),
            index_(0)
{
    timeStamp = timestamp;
    data      = static_cast<uint8_t*>(frame->data);
    size      = frame->size;
    fd        = frame->fd;
    metadata  = frame->metadata;
    index_    = index;
    mSocketFD = SocketFD;
}

ICameraClientFrame::~ICameraClientFrame()
{}


uint32_t ICameraClientFrame::acquireRef()
{
    refs_++;
}

uint32_t ICameraClientFrame::releaseRef()
{
    refs_--;
    if (refs_ == 0)
    {
        ICameraCommandType command;
        int sentBytes;

        memset(&command, 0, sizeof(ICameraCommandType));
        command.type = RELEASE_FRAME;
        command.payload_size = sizeof(index_);

        if (sentBytes != sizeof(ICameraCommandType))
        {
            return -1;
        }

        sentBytes = socket_sendmsg(mSocketFD, &index_, command.payload_size, 0);
    }
}

ICameraClientParams::ICameraClientParams():
                mSocketFD(0),
                mAPIlock(NULL),
                mSlaveDevice(false)
{
     pthread_cond_init(&mAPIcond, NULL);
}
ICameraClientParams::~ICameraClientParams()
{
}
void ICameraClientParams::InitComm(int socketFD, pthread_mutex_t *mAPIlock)
{
    mSocketFD = socketFD;
    mAPIlock = mAPIlock;
} 

void ICameraClientParams::SetSlaveDevice(bool SlaveDevice)
{
    mSlaveDevice = SlaveDevice;
}

int ICameraClientParams::init(ICameraDevice* device)
{
    ICameraCommandType command;
    int sentBytes;

    pthread_mutex_lock(mAPIlock);

    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = GET_PARAMETERS;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType),0);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(mAPIlock);
        return -1;
    }

    pthread_cond_wait(&mAPIcond, mAPIlock);

    pthread_mutex_unlock(mAPIlock);

    return 0;

}

int ICameraClientParams::commit()
{
    ICameraCommandType command;
    int sentBytes;
    char *paramStr = NULL;

    if (mSlaveDevice)
    {
        fprintf(stderr,"It is Slave device\n");
        return -1;
    }

    pthread_mutex_lock(mAPIlock);

    paramStr = (char *)toString().c_str();
    memset(&command, 0, sizeof(ICameraCommandType));

    command.type = SET_PARAMETERS;
    command.payload_size = strlen(paramStr);

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType),0);    
    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(mAPIlock);
        return -1;
    }

    sentBytes = socket_sendmsg(mSocketFD, &paramStr,command.payload_size,0);
    if (sentBytes != command.payload_size)
    {
        pthread_mutex_unlock(mAPIlock);
        return -1;
    }

    pthread_cond_wait(&mAPIcond, mAPIlock);

    pthread_mutex_unlock(mAPIlock);
    return 0;

}

ICameraClient::ICameraClient():
            mThread_id(0),
            mSocketFD(0),
            mPreviewRunnnig(false),
            mVideoRunning(false),
            mPictureStarted(0),
            mNumCameras(0),
            mCamListner(NULL),
            mError(0)
{
    sockaddr_un address;
    int res;
    int len;
    int i;

    memset (&mCameraFunc, 0, sizeof(CameraFuncType));
    mSocketFD = socket(AF_UNIX, SOCK_STREAM, 0);
    if (mSocketFD == -1)
    {
        fprintf(stderr,"ERROR creating socket\n");
        return;
    }

    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, SEVER_SOCKET_PATH);
    len = sizeof(address);
    res = connect(mSocketFD, (sockaddr*)&address, len);
    if(res == -1)
    {
        fprintf(stderr,"ERROR connecting socket\n");
        mSocketFD = -1;
        return;
    }
     pthread_cond_init(&mAPIcond, NULL);
     pthread_mutex_init(&mAPIlock, NULL);
     params.InitComm(mSocketFD, &mAPIlock);
     mIsActive = 1;
}

ICameraClient::~ICameraClient()
{
     pthread_cond_destroy(&mAPIcond);
     pthread_mutex_destroy(&mAPIlock);
     close(mSocketFD);
}

int ICameraClient::Init()
{
    ICameraCommandType command;
    int sentBytes;

    pthread_create(&mThread_id, NULL, MainThreadLoop, this);

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = GET_NUM_CAMERAS;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command, sizeof(ICameraCommandType),0);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    pthread_cond_wait(&mAPIcond, &mAPIlock);

    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = GET_CAMERAS_INFO;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command, sizeof(ICameraCommandType),0);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    pthread_cond_wait(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    return 0;
}

int ICameraClient::deInit()
{
    ICameraCommandType command;
    int sentBytes;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = STOP_SESSION;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command, sizeof(ICameraCommandType),0);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    pthread_cond_wait(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);

    pthread_join(mThread_id, NULL);
    return 0;
}

void ICameraClient::SetNumCameras(int NumCameras)
{
    mNumCameras = NumCameras;
}

int  ICameraClient::GetNumCameras()
{
    return mNumCameras;
}

void ICameraClient::SetCameraInfo(CameraFuncType *CameraFunc)
{
    memcpy(&mCameraFunc, CameraFunc, sizeof(CameraFuncType));
}

void  ICameraClient::GetCameraInfo( int idx, struct CameraInfo  *info)
{
   info->func = mCameraFunc.camera_func[idx];
}

int ICameraClient::OpenCamera(int camId)
{
    ICameraCommandType command;
    int sentBytes;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = OPEN_CAMERA;
    command.payload_size = sizeof(camId);

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType),0);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    sentBytes = socket_sendmsg(mSocketFD, &camId, command.payload_size, 0);

    if (sentBytes != 1)
    {
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    pthread_cond_wait(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    return 0;

}


int ICameraClient::CloseCamera(int camId)
{
    ICameraCommandType command;
    int sentBytes;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = CLOSE_CAMERA;
    command.payload_size = sizeof(camId);

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    sentBytes = socket_sendmsg(mSocketFD, &camId, command.payload_size, 0);

    if (sentBytes != 1)
    {
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    pthread_cond_wait(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    return 0;

}

int ICameraClient::StartPreview()
{
    ICameraCommandType command;
    int sentBytes;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = START_PREVIEW;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    pthread_cond_wait(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    return 0;    
}

int ICameraClient::StopPreview()
{
    ICameraCommandType command;
    int sentBytes;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = STOP_PREVIEW;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    fprintf(stderr, "DEBUG %s:%d\n", __func__, __LINE__);
    pthread_cond_wait(&mAPIcond, &mAPIlock);
    fprintf(stderr, "DEBUG %s:%d\n", __func__, __LINE__);

    pthread_mutex_unlock(&mAPIlock);
    return 0;
}


int ICameraClient::StartRecording()
{
    ICameraCommandType command;
    int sentBytes;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = START_RECORDING;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    pthread_cond_wait(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    return 0;    
}

int ICameraClient::StopRecording()
{
    ICameraCommandType command;
    int sentBytes;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = STOP_RECORDING;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    pthread_cond_wait(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    return 0;    
}


int ICameraClient::TakePicture()
{
    ICameraCommandType command;
    int sentBytes;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = TAKE_PICTURE;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    pthread_cond_wait(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    return 0;    
}

int ICameraClient::CancelPicture()
{
    ICameraCommandType command;
    int sentBytes;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = CANCEL_PICTURE;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    pthread_cond_wait(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    return 0;    
}

int ICameraClient::sendFaceDetectCommand(bool turn_on)
{
    ICameraCommandType command;
    int sentBytes;
    int payload = turn_on;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = OPEN_CAMERA;
    command.payload_size = sizeof(payload);

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    sentBytes = socket_sendmsg(mSocketFD, &payload, command.payload_size, 0);

    if (sentBytes != 1)
    {
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    pthread_cond_wait(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    return 0;

}

void ICameraClient::AddListener(ICameraListener* listener)
{

    if (mCamListner == listener) {
        fprintf(stderr,"this listener is already added\n");
        return;
    }


    mCamListner = listener;
}


void ICameraClient::RemoveListener(ICameraListener* listener)
{


    if (mCamListner == listener) {
       mCamListner = NULL;
    }

    if (mCamListner)
        fprintf(stderr,"No such listener\n");
}
