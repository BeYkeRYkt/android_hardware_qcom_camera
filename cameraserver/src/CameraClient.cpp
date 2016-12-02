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
    int err_no = 0;

    if (pCameraClient->mSocketFD > 0)
    {
      while (pCameraClient->GetIsActive())
      {

        readBytes = socket_recvmsg(pCameraClient->mSocketFD,
                  &CameraCommand, sizeof(ICameraCommandType),
                  &recvFD, &err_no);
        if(err_no != 0) {
             CameraCommand.type = STOP_SESSION_DONE;
             pthread_cond_signal(&pCameraClient->mAPIcond);
             break;
        }
        if ((CameraCommand.payload_size != 0)/* && (readBytes == sizeof(ICameraCommandType))*/)
        {

            commandPayload = (uint8_t *)malloc(CameraCommand.payload_size);
            if (commandPayload == NULL)
            {
                fprintf(stderr,"Fail to allocate memory for command payload\n");
                pthread_cond_signal(&pCameraClient->params.mAPIcond);
                break;
            }
            int totalBytes = 0;
            do {
                readBytes = socket_recvmsg(pCameraClient->mSocketFD,
                    (char*)commandPayload + totalBytes,
                    CameraCommand.payload_size - totalBytes,
                    &recvFD, &err_no);
                if(err_no != 0) {
                    CameraCommand.type = STOP_SESSION_DONE;
                    pthread_cond_signal(&pCameraClient->mAPIcond);
                    break;
                }
                totalBytes += readBytes;
            } while (readBytes > 0 && totalBytes < CameraCommand.payload_size);

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
                         pCameraClient->SetError(ERROR_GET_PARAMS);
                    }
                    pCameraClient->SetError(NO_ERROR);
                    pthread_cond_signal(&pCameraClient->params.mAPIcond);
                    break;
                }
                case GET_NUM_CAMERAS_DONE:
                {
                    pCameraClient->SetNumCameras(*(int *)commandPayload);
                    pCameraClient->SetError(NO_ERROR);
                    pthread_cond_signal(&pCameraClient->mAPIcond);
                    break;
                }
                case GET_CAMERAS_INFO_DONE:
                    pCameraClient->SetCameraInfo((CameraFuncType *)commandPayload);
                    pCameraClient->SetError(NO_ERROR);
                    pthread_cond_signal(&pCameraClient->mAPIcond);
                    break;
                case SET_PARAMETERS_DONE:
                    pCameraClient->SetError(NO_ERROR);
                    pthread_cond_signal(&pCameraClient->params.mAPIcond);
                    break;
                case OPEN_CAMERA_DONE:
                    pCameraClient->SetClientMode(*(ClientModeType *)commandPayload);
                    pCameraClient->SetError(NO_ERROR);
                    pthread_cond_signal(&pCameraClient->mAPIcond);
                    break;
                case STOP_PREVIEW_DONE:
                case ENABLE_FACE_DETECT_DONE:
                case START_PREVIEW_DONE:
                case START_RECORDING_DONE:
                case STOP_RECORDING_DONE:
                case TAKE_PICTURE_DONE:
                case CANCEL_PICTURE_DONE:
                    pCameraClient->SetError(NO_ERROR);
                    pthread_cond_signal(&pCameraClient->mAPIcond);
                    break;
                case RELEASE_FRAME_DONE:
                    pCameraClient->SetError(NO_ERROR);
                    break;
                case NEW_PREVIEW_FRAME:
                {
                    CameraMemory *pnewMemEntry;
                    ICameraClientFrame *pPreviewFrame;
                    ICameraCommandFrameType *pnewFrame =
                     (ICameraCommandFrameType *)commandPayload;
                    if (recvFD > 0)
                    {
                        pnewMemEntry = new CameraMemory(recvFD, pnewFrame->bufSize);
                        pCameraClient->pMem.push_back(pnewMemEntry);
                    }
                    pPreviewFrame = new ICameraClientFrame(pnewFrame->timestamp,
                            &pCameraClient->pMem[pnewFrame->index]->frame,
                            pnewFrame->index,
                            pCameraClient->mSocketFD,
                            pCameraClient->GetLock());
                    pPreviewFrame->facedata = NULL;
                    pPreviewFrame->acquireRef();
                    pCameraClient->mCamListner->onPreviewFrame(static_cast<ICameraFrame*>(pPreviewFrame));

                    break;
                }
                case NEW_SNAPSHOT_FRAME:
                {
                    CameraMemory *pnewMemEntry;
                    ICameraClientFrame *pSnapshotFrame;
                    ICameraCommandFrameType *pnewFrame =
                        (ICameraCommandFrameType *)commandPayload;
                    pnewMemEntry = new CameraMemory(recvFD, pnewFrame->bufSize);
                    pSnapshotFrame = new ICameraClientFrame(0,
                            &pnewMemEntry->frame,
                            pnewFrame->index,
                            pCameraClient->mSocketFD,
                            pCameraClient->GetLock());

                    pSnapshotFrame->facedata = NULL;
                    pSnapshotFrame->acquireRef();
                    pCameraClient->mCamListner->onPictureFrame(pSnapshotFrame);

                    break;
                }
                case NEW_VIDEO_FRAME:
                {
                    CameraMemory *pnewMemEntry;
                    ICameraClientFrame *pVideoFrame;
                    ICameraCommandFrameType *pnewFrame =
                     (ICameraCommandFrameType *)commandPayload;
                    if (recvFD > 0)
                    {
                        pnewMemEntry = new CameraMemory(recvFD, pnewFrame->bufSize);
                        pCameraClient->pMem.push_back(pnewMemEntry);
                    }

                    pVideoFrame = new ICameraClientFrame(pnewFrame->timestamp,
                            &pCameraClient->pMem[pnewFrame->index]->frame,
                            pnewFrame->index,
                            pCameraClient->mSocketFD,
                            pCameraClient->GetLock());
                    pVideoFrame->facedata = NULL;
                    pVideoFrame->acquireRef();
                    pCameraClient->mCamListner->onVideoFrame(static_cast<ICameraFrame*>(pVideoFrame));

                    break;
                }
                case NEW_META_FRAME:
                {
                    CameraMemory *pnewMemEntry;
                    ICameraClientFrame *pMetaFrame;
                    FaceRoi *FaceROI = (FaceRoi *)commandPayload;

                    pMetaFrame = new ICameraClientFrame(0, -1,
                            0, NULL, NULL, -1, pCameraClient->mSocketFD,
                            pCameraClient->GetLock());
                    pMetaFrame->facedata = FaceROI;
                    pMetaFrame->acquireRef();
                    pCameraClient->mCamListner->onMetadataFrame(static_cast<ICameraFrame*>(pMetaFrame));
                    break;
                }
                case STOP_SESSION_DONE:
                    pCameraClient->SetIsActive(0);
                    pCameraClient->SetError(NO_ERROR);
                    pthread_cond_signal(&pCameraClient->mAPIcond);
                    break;
                case SERVER_NOTIFICATION:
                    pCameraClient->SetError(*(ErrorType *)commandPayload);
                    pthread_cond_signal(&pCameraClient->mAPIcond);

                    if (pCameraClient->GetError() == ERROR_SERVER_DIED)
                    {
                        pCameraClient->mCamListner->onError();
                    }
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

int wait_for_ack(pthread_cond_t *mAPIcond, pthread_mutex_t *mAPIlock)
{
    struct timespec time;
    int rc = 0;

    clock_gettime(CLOCK_REALTIME, &time);
    time.tv_sec += 1;

    rc = pthread_cond_timedwait(mAPIcond, mAPIlock, &time);

    return rc;
}

ICameraClientFrame::ICameraClientFrame( int64_t timestamp,
            QCamera2Frame *frame, int index, int SocketFD,
            pthread_mutex_t *pAPIlock) :
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
    mAPIlock  = pAPIlock;
}

ICameraClientFrame::ICameraClientFrame( int64_t timestamp,
            int fd, int bufsize, void *data, void *metadata,
            int index, int SocketFD,
            pthread_mutex_t *pAPIlock) :
            refs_(0),
            index_(0)
{
    timeStamp = timestamp;
    data      = static_cast<uint8_t*>(data);
    size      = bufsize;
    fd        = fd;
    metadata  = metadata;
    index_    = index;
    mSocketFD = SocketFD;
    mAPIlock  = pAPIlock;
}

ICameraClientFrame::~ICameraClientFrame()
{

}


uint32_t ICameraClientFrame::acquireRef()
{
    refs_++;
}

uint32_t ICameraClientFrame::releaseRef()
{
    int err_no = 0;
    refs_--;
    if (refs_ == 0)
    {
            ICameraCommandType command;
            int sentBytes;

            pthread_mutex_lock(mAPIlock);

            memset(&command, 0, sizeof(ICameraCommandType));
            command.type = RELEASE_FRAME;
            command.payload_size = sizeof(index_);

            sentBytes = socket_sendmsg(mSocketFD, &command, sizeof(command), 0, &err_no);
            sentBytes = socket_sendmsg(mSocketFD, &index_, command.payload_size, 0, &err_no);

            if (sentBytes != sizeof(ICameraCommandType))
            {
                pthread_mutex_unlock(mAPIlock);
                return -1;
            }

            pthread_mutex_unlock(mAPIlock);

        delete this;
    }
}

ICameraClientParams::ICameraClientParams():
                mSocketFD(0),
                mSlaveMode(false),
                dev(NULL)
{
     pthread_cond_init(&mAPIcond, NULL);
}
ICameraClientParams::~ICameraClientParams()
{
     pthread_cond_destroy(&mAPIcond);
}
void ICameraClientParams::InitComm(int socketFD, pthread_mutex_t *pAPIlock)
{
    mSocketFD = socketFD;
    mAPIlock = pAPIlock;
}

void ICameraClientParams::SetSlaveMode(bool SlaveMode)
{
    mSlaveMode = SlaveMode;
}

void ICameraClientParams::SetDevice(ICameraClient *device)
{
    dev = device;
}

int ICameraClientParams::init(ICameraDevice* device)
{
    ICameraCommandType command;
    int sentBytes;
    int err_no = 0, rc = 0;

    pthread_mutex_lock(mAPIlock);

    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = GET_PARAMETERS;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command,
                        sizeof(ICameraCommandType),0, &err_no);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        dev->SetError(ERROR_SEND_COMMAND);
        pthread_mutex_unlock(mAPIlock);
        return -1;
    }

    rc = wait_for_ack(&mAPIcond, mAPIlock);

    pthread_mutex_unlock(mAPIlock);

    if (rc == ETIMEDOUT) {
        fprintf(stderr,"Parameters Init Ack was not received\n");
        return -1;
    }

    if (dev->GetError() != NO_ERROR) {
        fprintf(stderr,"Error during parameters init \n");
        return -1;
    }
    return 0;

}

int ICameraClientParams::commit()
{
    ICameraCommandType command;
    int sentBytes;
    char *paramStr = NULL;
    int err_no = 0, rc = 0;

    if (mSlaveMode)
    {
        dev->SetError(ERROR_COMMIT_PARAMS);
        fprintf(stderr,"It is Slave device\n");
        return -1;
    }

    pthread_mutex_lock(mAPIlock);

    paramStr = (char *)toString().c_str();
    memset(&command, 0, sizeof(ICameraCommandType));

    command.type = SET_PARAMETERS;
    command.payload_size = strlen(paramStr);

    sentBytes = socket_sendmsg(mSocketFD, &command,
                        sizeof(ICameraCommandType),0, &err_no);
    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(mAPIlock);
        dev->SetError(ERROR_SEND_COMMAND);
        return -1;
    }

    sentBytes = socket_sendmsg(mSocketFD, paramStr, command.payload_size,0, &err_no);
    if (sentBytes != command.payload_size)
    {
        pthread_mutex_unlock(mAPIlock);
        dev->SetError(ERROR_SEND_COMMAND);
        return -1;
    }

    rc = wait_for_ack(&mAPIcond, mAPIlock);

    pthread_mutex_unlock(mAPIlock);

    if (rc == ETIMEDOUT) {
        fprintf(stderr, "Commit Ack was not received \n");
        return -1;
    }
    if (dev->GetError() != NO_ERROR) {
        fprintf(stderr, "Error during commit\n");
        return -1;
    }
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
            mError(NO_ERROR),
            mClientMode(MASTER_CLIENT)
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
    int err_no = 0, rc = 0;

    pthread_create(&mThread_id, NULL, MainThreadLoop, this);

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = GET_NUM_CAMERAS;
    command.payload_size = 0;

    params.SetDevice(this);

    sentBytes = socket_sendmsg(mSocketFD, &command, sizeof(ICameraCommandType),0, &err_no);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        mError = ERROR_SEND_COMMAND;
        return -1;
    }

    rc = wait_for_ack(&mAPIcond, &mAPIlock);
    if (rc == ETIMEDOUT) {
        fprintf(stderr,"Client Init Ack was not received\n");
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }
    if (mError != NO_ERROR) {
        fprintf(stderr,"Error during client init\n");
        pthread_mutex_unlock(&mAPIlock);
        return -1;
    }

    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = GET_CAMERAS_INFO;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command, sizeof(ICameraCommandType),0, &err_no);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        mError = ERROR_SEND_COMMAND;
        return -1;
    }

    rc = wait_for_ack(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);

    if (rc == ETIMEDOUT) {
        fprintf(stderr,"Client Init Ack was not received\n");
        return -1;
    }
    if (mError != NO_ERROR)
        return -1;

    return 0;
}

int ICameraClient::deInit()
{
    ICameraCommandType command;
    int sentBytes;
    int err_no = 0, rc = 0;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = STOP_SESSION;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command, sizeof(ICameraCommandType),0, &err_no);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        mError = ERROR_SEND_COMMAND;
        return -1;
    }

    rc = wait_for_ack(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);

    if (rc ==  ETIMEDOUT) {
        fprintf(stderr, " Client Deinit Ack was not received\n");
    }
    if (mError != NO_ERROR) {
        fprintf(stderr," Error during client deinit\n");
        return -1;
    }

    pthread_join(mThread_id, NULL);
    return 0;
}

void ICameraClient::SetNumCameras(int NumCameras)
{
    mNumCameras = NumCameras;
}

int  ICameraClient::getNumberOfCameras()
{
    return mNumCameras;
}

void ICameraClient::SetCameraInfo(CameraFuncType *CameraFunc)
{
    memcpy(&mCameraFunc, CameraFunc, sizeof(CameraFuncType));
}

void  ICameraClient::getCameraInfo( int idx, struct CameraInfo  *info)
{
   info->func = mCameraFunc.camera_func[idx];
}

void ICameraClient::SetClientMode(ClientModeType ClientMode)
{
     mClientMode = ClientMode;
     if (mClientMode == SLAVE_CLIENT)
        params.SetSlaveMode(true);
     fprintf(stderr, "Master mode %d\n", mClientMode);
}

int ICameraClient::openCamera(int camId)
{
    ICameraCommandType command;
    int sentBytes;
    int err_no = 0, rc = 0;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = OPEN_CAMERA;
    command.payload_size = sizeof(camId);

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType),0, &err_no);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        mError = ERROR_SEND_COMMAND;
        return -1;
    }

    sentBytes = socket_sendmsg(mSocketFD, &camId, command.payload_size, 0, &err_no);

    if (sentBytes != 1)
    {
        pthread_mutex_unlock(&mAPIlock);
        mError = ERROR_SEND_COMMAND;
        return -1;
    }

    rc = wait_for_ack(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    if (rc == ETIMEDOUT) {
        fprintf(stderr, " Client OpenCamera Ack was not received\n");
        return -1;
    }
    if (mError != NO_ERROR) {
        fprintf(stderr, "Error during Client OpenCamera\n");
        return -1;
    }
    return 0;

}


int ICameraClient::closeCamera(int camId)
{
    ICameraCommandType command;
    int sentBytes;
    int err_no = 0, rc = 0;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = CLOSE_CAMERA;
    command.payload_size = sizeof(camId);

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0, &err_no);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        mError = ERROR_SEND_COMMAND;
        return -1;
    }

    sentBytes = socket_sendmsg(mSocketFD, &camId, command.payload_size, 0, &err_no);

    if (sentBytes != 1)
    {
        pthread_mutex_unlock(&mAPIlock);
        mError = ERROR_SEND_COMMAND;
        return -1;
    }

    rc = wait_for_ack(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    if (rc ==  ETIMEDOUT) {
        fprintf(stderr," Client CloseCamer Ack was not received\n");
        return -1;
    }
    if (mError != NO_ERROR) {
        fprintf(stderr," Error during client closecamera\n");
        return -1;
    }
    return 0;

}

int ICameraClient::startPreview()
{
    ICameraCommandType command;
    int sentBytes;
    int err_no = 0, rc = 0;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = START_PREVIEW;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0, &err_no);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        mError = ERROR_SEND_COMMAND;
        return -1;
    }

    rc = wait_for_ack(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    if (rc == ETIMEDOUT) {
        fprintf(stderr,"Client StartPreview Ack was not received\n");
        return -1;
    }
    if (mError != NO_ERROR){
        fprintf(stderr,"Error during Client StartPreview \n");
        return -1;
    }
    return 0;
}

int ICameraClient::stopPreview()
{
    ICameraCommandType command;
    int sentBytes;
    int err_no = 0, rc = 0;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = STOP_PREVIEW;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0, &err_no);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        mError = ERROR_SEND_COMMAND;
        return -1;
    }

    rc = wait_for_ack(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    if (rc ==  ETIMEDOUT) {
        fprintf(stderr,"Client StopPreview was not received \n");
        return -1;
    }
    if (mError != NO_ERROR) {
        fprintf(stderr,"Error during client stop preview \n");
        return -1;
    }
    return 0;
}


int ICameraClient::startRecording()
{
    ICameraCommandType command;
    int sentBytes;
    int err_no = 0, rc = 0;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = START_RECORDING;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0, &err_no);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        mError = ERROR_SEND_COMMAND;
        return -1;
    }

    rc = wait_for_ack(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    if (rc ==  ETIMEDOUT) {
        fprintf(stderr,"Client StartRecording Ack was not recieved\n");
        return -1;
    }
    if (mError != NO_ERROR) {
        fprintf(stderr," Error duirng client Start Recording \n");
        return -1;
    }
    return 0;
}

int ICameraClient::stopRecording()
{
    ICameraCommandType command;
    int sentBytes;
    int err_no = 0, rc = 0;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = STOP_RECORDING;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0, &err_no);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        mError = ERROR_SEND_COMMAND;
        return -1;
    }

    rc = wait_for_ack(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    if (rc == ETIMEDOUT) {
        fprintf(stderr, "Client StopRecording Ack was not received\n");
    }
    if (mError != NO_ERROR) {
        fprintf(stderr,"Error during client stop recording\n");
        return -1;
    }
    return 0;
}


int ICameraClient::takePicture()
{
    ICameraCommandType command;
    int sentBytes;
    int err_no = 0, rc = 0;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = TAKE_PICTURE;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0, &err_no);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        mError = ERROR_SEND_COMMAND;
        return -1;
    }

    rc = wait_for_ack(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    if (rc == ETIMEDOUT) {
        fprintf(stderr,"Client TakePicture Ack was not received\n");
        return -1;
    }
    if (mError != NO_ERROR) {
        fprintf(stderr,"Error during client take picture\n");
        return -1;
    }
    return 0;
}

int ICameraClient::cancelPicture()
{
    ICameraCommandType command;
    int sentBytes;
    int err_no = 0, rc = 0;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = CANCEL_PICTURE;
    command.payload_size = 0;

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0, &err_no);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        mError = ERROR_SEND_COMMAND;
        return -1;
    }

    rc = wait_for_ack(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    if (rc == ETIMEDOUT) {
        fprintf(stderr," Client CancelPicture Ack was not received\n");
        return -1;
    }
    if (mError != NO_ERROR) {
        fprintf(stderr,"Error during client cancel picture\n");
        return -1;
    }
    return 0;
}

int ICameraClient::sendFaceDetectCommand(bool turn_on)
{
    ICameraCommandType command;
    int sentBytes;
    int payload = turn_on;
    int err_no = 0, rc = 0;

    pthread_mutex_lock(&mAPIlock);
    memset(&command, 0, sizeof(ICameraCommandType));
    command.type = ENABLE_FACE_DETECT;
    command.payload_size = sizeof(payload);

    sentBytes = socket_sendmsg(mSocketFD, &command,sizeof(ICameraCommandType), 0, &err_no);

    if (sentBytes != sizeof(ICameraCommandType))
    {
        pthread_mutex_unlock(&mAPIlock);
        mError = ERROR_SEND_COMMAND;
        return -1;
    }

    sentBytes = socket_sendmsg(mSocketFD, &payload, command.payload_size, 0, &err_no);

    if (sentBytes != 1)
    {
        pthread_mutex_unlock(&mAPIlock);
        mError = ERROR_SEND_COMMAND;
        return -1;
    }

    rc = wait_for_ack(&mAPIcond, &mAPIlock);

    pthread_mutex_unlock(&mAPIlock);
    if (rc == ETIMEDOUT) {
        fprintf(stderr," Client sendFaceDetect Ack was not received\n");
        return -1;
    }
    if (mError != NO_ERROR) {
        fprintf(stderr,"Error during client sendFaceDetect\n");
        return -1;
    }
    return 0;

}

void ICameraClient::addListener(ICameraListener* listener)
{

    if (mCamListner == listener) {
        fprintf(stderr,"this listener is already added\n");
        return;
    }


    mCamListner = listener;
}


void ICameraClient::removeListener(ICameraListener* listener)
{


    if (mCamListner == listener) {
       mCamListner = NULL;
    }

    if (mCamListner)
        fprintf(stderr,"No such listener\n");
}
