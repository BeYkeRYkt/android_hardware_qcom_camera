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
#include <sys/signal.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <sstream>
#include "camera.h"
#include "CameraServer.hpp"
#include "CameraClientUtil.hpp"
#include <algorithm>

#define CAM_SERVER_SOCKET "/root/cam_srv_sock"

#define XNO_ERROR 0 //REPLACE
using namespace camera;

ClientDescriptor::ClientDescriptor(int cFd):
                mClientFD(cFd),
                mNumFrameFd(0),
                mState(CLIENT_INACTIVE),
                mExpPicture(false)
{
    pthread_mutex_init(&mClientLock, NULL);
    // Create internal communication pipe
    if(pipe(mIntCommPipeFd) < 0) {
        fprintf(stderr, "Error creating pipe\n");
    }
}

ClientDescriptor::~ClientDescriptor()
{
    close(mIntCommPipeFd[0]);
    close(mIntCommPipeFd[1]);
    pthread_mutex_destroy(&mClientLock);
}

CameraObject::CameraObject():
            mOpenInstances(0),
            mStartedPreviews(0)
{
    return;
}

CameraObject::~CameraObject()
{
    return;
}

void CameraObject::onError()
{
    return;
}

void CameraObject::onPreviewFrame(ICameraFrame* frame)
{
    return;
}

void CameraObject::onVideoFrame(ICameraFrame* frame)
{
    return;
}

void CameraObject::onPictureFrame(ICameraFrame* frame)
{
    return;
}

void CameraObject::onMetadataFrame(ICameraFrame *frame)
{
    return;
}

SrvCameraObject::SrvCameraObject(CameraServer *srv, int camId):
            mServer(srv),
            mCamId(camId)
{
}

SrvCameraObject::~SrvCameraObject()
{
}


void SrvCameraObject::onError()
{
}

void SrvCameraObject::onPreviewFrame(ICameraFrame* frame)
{
    mServer->dispatchFrame(mCamId, NEW_PREVIEW_FRAME, frame);
}

void SrvCameraObject::onVideoFrame(ICameraFrame* frame)
{
    mServer->dispatchFrame(mCamId, NEW_VIDEO_FRAME, frame);
}

void SrvCameraObject::onPictureFrame(ICameraFrame* frame)
{
    mServer->dispatchFrame(mCamId, NEW_SNAPSHOT_FRAME, frame);
}

void SrvCameraObject::onMetadataFrame(ICameraFrame *frame)
{
    mServer->dispatchFrame(mCamId, NEW_META_FRAME, frame);
}

struct ThreadData {
  ClientDescriptor *client;
  CameraServer     *server;
};

void *cleanup_thread(void *arg)
{
    int              *cleanUpPipeFd = (int*)arg;
    bool              activeFlag = true;
    void             *buf;
    ClientDescriptor *cliDesc;

    while (activeFlag) {
        read(cleanUpPipeFd[0], &buf, sizeof(buf));
        cliDesc = (ClientDescriptor*) buf;

        if (cliDesc) {
            pthread_join(cliDesc->mThread, NULL);
            close(cliDesc->mClientFD);
            delete(cliDesc);
        } else {
            // Signal for completion
            activeFlag = 0;
        }
    }
    return NULL;
}

void *client_thread(void *arg)
{
    ThreadData data = *(ThreadData*)arg;
    bool activeFlag = true;
    ICameraCommandType inc_command;
    ICameraCommandType out_command;
    ICameraCommandType err_command;
    void *payload = NULL;
    void *out_payload = NULL;
    int fd = 0, err_no = 0, nfds;
    ClientThreadState threadState = WAIT_RECV_COMMAND;
    int intCmd;
    ErrorType error = NO_ERROR;

    err_command.payload_size = sizeof(error);
    err_command.type = SERVER_NOTIFICATION;

    while (activeFlag) {
        int readyFds;
        fd_set fdSetRead;
        fd_set fdSetWrite;
        int nfds = (data.client->mClientFD > data.client->mIntCommPipeFd[0]) ?
                data.client->mClientFD:data.client->mIntCommPipeFd[0];

        FD_ZERO(&fdSetRead);
        FD_ZERO(&fdSetWrite);
        if (threadState == WAIT_RECV_COMMAND || threadState == WAIT_RECV_PAYLOAD) {
            FD_SET(data.client->mClientFD, &fdSetRead);
        } else {
            FD_SET(data.client->mClientFD, &fdSetWrite);
        }

        FD_SET(data.client->mIntCommPipeFd[0], &fdSetRead);
        readyFds = select (nfds + 1, &fdSetRead, &fdSetWrite, NULL, NULL);
        if (readyFds <= 0) {
            continue;
        }
        if (FD_ISSET(data.client->mIntCommPipeFd[0], &fdSetRead)) {
            // Server notification received
            read(data.client->mIntCommPipeFd[0], &intCmd, sizeof(intCmd));
            error = ERROR_SERVER_DIED;
            socket_sendmsg(data.client->mClientFD, &err_command,
                    sizeof(err_command), fd, &err_no);
            socket_sendmsg(data.client->mClientFD, &error,
                    sizeof(error), fd, &err_no);
            break;
        }
        if (!FD_ISSET(data.client->mClientFD, &fdSetRead) && !FD_ISSET(data.client->mClientFD, &fdSetWrite)) {
            // Continue waiting for client request
            continue;
        }

        switch (threadState) {
        case WAIT_RECV_COMMAND: {
            // Receive cmd msg from client
            socket_recvmsg(data.client->mClientFD, &inc_command,
                    sizeof(inc_command), &fd, &err_no);
            if (inc_command.payload_size) {
                threadState = WAIT_RECV_PAYLOAD;
            } else {
                threadState = WAIT_SEND_COMMAND;
            }
        }
            break;
        case WAIT_RECV_PAYLOAD: {
            payload = malloc(inc_command.payload_size);
            socket_recvmsg(data.client->mClientFD, payload,
                    inc_command.payload_size, &fd, &err_no);
            threadState = WAIT_SEND_COMMAND;

        }
            break;
        case WAIT_SEND_COMMAND: {
            // send acknowledge
            pthread_mutex_lock(&data.client->mClientLock);
            socket_sendmsg(data.client->mClientFD, &out_command,
                    sizeof(out_command), fd, &err_no);
            if (out_command.payload_size) {
                threadState = WAIT_SEND_PAYLOAD;
            } else {
                threadState = WAIT_RECV_COMMAND;
                pthread_mutex_unlock(&data.client->mClientLock);
            }
        }
            break;
        case WAIT_SEND_PAYLOAD: {
            socket_sendmsg(data.client->mClientFD, out_payload,
                    out_command.payload_size, fd, &err_no);
            threadState = WAIT_RECV_COMMAND;
            pthread_mutex_unlock(&data.client->mClientLock);
        }
            break;
        default:
            break;
        }


        if (out_command.type == STOP_SESSION_DONE &&
                threadState == WAIT_RECV_COMMAND) {
            // We'd just acknowledged "stop session".
            // Now stop listening and proceed to cleanup
            activeFlag = false;
        }

        if (err_no != 0) {
            fprintf(stderr, "Client communication broken, terminating...\n");
            if (threadState == WAIT_SEND_PAYLOAD) {
                pthread_mutex_unlock(&data.client->mClientLock);
            }
            activeFlag = false;
        }

        if (!err_no && threadState == WAIT_SEND_COMMAND) {
            // process command
            error = data.server->processCommand(*data.client, inc_command, payload,
                    out_command, &out_payload);
            if (error != NO_ERROR) {
                socket_sendmsg(data.client->mClientFD, &err_command,
                        sizeof(err_command), fd, &err_no);
                socket_sendmsg(data.client->mClientFD, &error,
                        sizeof(error), fd, &err_no);
            }

        }

        if (payload && threadState == WAIT_SEND_COMMAND) {
            free(payload);
            payload = NULL;
            inc_command.payload_size = 0;
        }
        if (out_payload && threadState == WAIT_RECV_COMMAND) {
            free(out_payload);
            out_payload = NULL;
            out_command.payload_size = 0;
        }
    }

    data.server->delClientByFD(data.client->mClientFD);

    return NULL;
}

CameraServer::CameraServer () : mSocket(-1)
{
    // Collect camera info
    mNumberOfCameras = getNumberOfCameras();
    fprintf(stderr, "Detected cameras %d\n", mNumberOfCameras);
    // Note - the below vectors sizes are determined here and should
    //        not be changed
    mCameraInfo.resize(mNumberOfCameras);
    pCamObjects.resize(mNumberOfCameras);

    for (int i = 0; i < mNumberOfCameras; i++) {
        getCameraInfo(i, mCameraInfo[i]);
        pCamObjects[i] = NULL;
    }
    mCleanupPipeFd[0] = -1;
    mCleanupPipeFd[1] = -1;
}

CameraServer::~CameraServer()
{
}


int  CameraServer::openCamera(int camId, int &isMaster)
{
    int rc;
    if (pCamObjects[camId]) {
        isMaster = false;
        pCamObjects[camId]->mOpenInstances++;
        return XNO_ERROR;
    }
    isMaster = true;
    pCamObjects[camId] = new SrvCameraObject(this, camId);
    rc = ICameraDevice::createInstance(camId, &pCamObjects[camId]->mCamera);
    if (rc != 0) {
        printf("could not open camera %d\n", camId);
        return rc;
    }
    pCamObjects[camId]->mOpenInstances++;
    pCamObjects[camId]->mCamera->addListener(pCamObjects[camId]);

    rc = pCamObjects[camId]->mParams.init(pCamObjects[camId]->mCamera);
    if (rc != 0) {
        printf("failed to init parameters\n");
        ICameraDevice::deleteInstance(&pCamObjects[camId]->mCamera);
        return rc;
    }
    return XNO_ERROR;
}

int  CameraServer::startPreview(int camId)
{
    int error;
    if (!pCamObjects[camId]->mOpenInstances) {
        return -1;
    }
    error = pCamObjects[camId]->mCamera->startPreview();
    pCamObjects[camId]->mStartedPreviews++;
    return error;
}

int  CameraServer::stopPreview(int camId)
{
    int error;
    if (!pCamObjects[camId]->mOpenInstances) {
        return -1;
    }
    if (pCamObjects[camId]->mStartedPreviews) {
        pCamObjects[camId]->mStartedPreviews--;
    } else {
        // Error - preview is stopped
        /*return*/
    }

    pCamObjects[camId]->mCamera->stopPreview();
    return error;
}

int  CameraServer::startRecording(int camId)
{
    int error;
    if (!pCamObjects[camId]->mOpenInstances) {
        return -1;
    }
    error = pCamObjects[camId]->mCamera->startRecording();
    return error;
}

int  CameraServer::stopRecording(int camId)
{
    int error;
    if (!pCamObjects[camId]->mOpenInstances) {
        return -1;
    }
    pCamObjects[camId]->mCamera->stopRecording();
    return error;
}

int  CameraServer::closeCamera(int camId)
{


    if (pCamObjects[camId]->mOpenInstances) {
        pCamObjects[camId]->mOpenInstances--;
    } else {
        // Error - camera is closed
        /*return*/
    }
    if (pCamObjects[camId]->mOpenInstances == 0) {
        pCamObjects[camId]->mCamera->removeListener(pCamObjects[camId]);
        ICameraDevice::deleteInstance(&pCamObjects[camId]->mCamera);
        delete (pCamObjects[camId]);
        pCamObjects[camId] = NULL;
    }
    return XNO_ERROR;
}

int  CameraServer::getParameters(int camId, int *paramSize, char **paramString)
{
    int error = 0;

    if (!pCamObjects[camId]->mOpenInstances) {
        return -1;
    }
    error = pCamObjects[camId]->mCamera->getParameters(0, 0, paramSize);

    if (!error) {
      *paramString = (char *)calloc(*paramSize+1, 1);
      if (NULL == *paramString) {
          error = -1;
      }
    }

    if (!error) {
      error = pCamObjects[camId]->mCamera->getParameters((uint8_t*)*paramString, *paramSize);
    }
    if (error) {
        free(*paramString);
        *paramString = NULL;
        *paramSize = 0;
    }

    return error;
}

int  CameraServer::setParameters(int camId,  char *paramString)
{
    int error = 0;

    if (!pCamObjects[camId]->mOpenInstances) {
        return -1;
    }
    pCamObjects[camId]->mParams.readObject(paramString);
    error = pCamObjects[camId]->mCamera->setParameters(pCamObjects[camId]->mParams);
    pCamObjects[camId]->mParams.commit();
    return XNO_ERROR;
}

int  CameraServer::takePicture(int camId)
{
    int error;
    if (!pCamObjects[camId]->mOpenInstances) {
        return -1;
    }
    error = pCamObjects[camId]->mCamera->takePicture();
    return XNO_ERROR;
}

int  CameraServer::cancelPicture(int camId)
{
    if (!pCamObjects[camId]->mOpenInstances) {
        return -1;
    }
    return XNO_ERROR;
}

int  CameraServer::enableFaceDetect(int camId, bool enable)
{
    if (!pCamObjects[camId]->mOpenInstances) {
        return -1;
    }
    pCamObjects[camId]->mCamera->sendFaceDetectCommand(enable);
    return XNO_ERROR;
}

ErrorType CameraServer::processCommand(ClientDescriptor &client,
        const ICameraCommandType &cmd, void* payload,
        ICameraCommandType &ack, void **ack_payload)
{
    int error = XNO_ERROR;
    ErrorType res = NO_ERROR;
    *ack_payload = NULL;
    ack.payload_size = 0;
    switch (cmd.type) {
        case GET_NUM_CAMERAS: {
            int *numCameras;
            ack.type = GET_NUM_CAMERAS_DONE;
            ack.payload_size = sizeof(int);
            *ack_payload = malloc(ack.payload_size);
            numCameras = (int*)(*ack_payload);
            *numCameras = mNumberOfCameras;
        }
            break;

        case GET_CAMERAS_INFO: {
            CameraFuncType *cameraFunc;
            ack.type = GET_CAMERAS_INFO_DONE;
            ack.payload_size = sizeof(CameraFuncType);
            *ack_payload = malloc(ack.payload_size);
            cameraFunc = (CameraFuncType *)(*ack_payload);
            for (int i = 0; i < mNumberOfCameras; i++) {
                cameraFunc->camera_func[i] =  mCameraInfo[i].func;
            }
        }
            break;

        case OPEN_CAMERA: {
            int *camId = (int*)payload;
            int *isMaster;
            client.mCameraId = *camId;
            error = openCamera(client.mCameraId, client.mMaster);
            if (!error) {
                client.mState = CLIENT_OPEN;
                ack.type = OPEN_CAMERA_DONE;
                ack.payload_size = sizeof(int);
                *ack_payload = malloc(ack.payload_size);
                isMaster = (int*)(*ack_payload);
                *isMaster = client.mMaster;
            } else {
                res = ERROR_OPEN_CAMERA;
            }
        }
            break;

        case CLOSE_CAMERA: {
            error = closeCamera(client.mCameraId);
            if (!error) {
                client.mState = CLIENT_INACTIVE;
                ack.type = CLOSE_CAMERA_DONE;
            } else {
                res = ERROR_CLOSE_CAMERA;
            }
        }
            break;

        case GET_PARAMETERS: {
            char *paramStr;
            error = getParameters(client.mCameraId, &ack.payload_size, &paramStr);
            if (!error) {
                *ack_payload = paramStr;
                ack.type = GET_PARAMETERS_DONE;
            } else {
                res = ERROR_GET_PARAMS;
            }
        }
            break;

        case SET_PARAMETERS: {
            char *paramStr = (char*)payload;
            if (cmd.payload_size) {
                error = setParameters(client.mCameraId, paramStr);
            } else {
                error = -1;
            }
            if (!error) {
                ack.type = SET_PARAMETERS_DONE;
            } else {
                res = ERROR_COMMIT_PARAMS;
            }

        }
            break;

        case ENABLE_FACE_DETECT: {
            bool enable = (0 != *(int *)payload);
            // extract enable from command
            error = enableFaceDetect(client.mCameraId, enable);
            if (!error) {
                ack.type = ENABLE_FACE_DETECT_DONE;
            } else {
                res = ERROR_FACE_DETECT;
            }
        }
            break;

        case START_PREVIEW: {
            error = startPreview(client.mCameraId);
            if (!error) {
                client.mState = CLIENT_PREVIEW;
                ack.type = START_PREVIEW_DONE;
            } else {
                res = ERROR_START_PREVIEW;
            }
        }
            break;

        case STOP_PREVIEW: {
            error = stopPreview(client.mCameraId);
            if (!error) {
                client.mState = CLIENT_OPEN;
                ack.type = STOP_PREVIEW_DONE;
            } else {
                res = ERROR_STOP_PREVIEW;
            }
        }
            break;

        case START_RECORDING: {
            error = startRecording(client.mCameraId);
            if (!error) {
                ack.type = START_RECORDING_DONE;
            } else {
                res = ERROR_START_RECORDING;
            }
        }
            break;

        case STOP_RECORDING: {
            error = stopRecording(client.mCameraId);
            if (!error) {
                ack.type = STOP_RECORDING_DONE;
            } else {
                res = ERROR_STOP_RECORDING;
            }
        }
            break;

        case TAKE_PICTURE: {
            client.mExpPicture = true;
            error = takePicture(client.mCameraId);
            if (!error) {
                ack.type = TAKE_PICTURE_DONE;
            } else {
                res = ERROR_TAKE_PICTURE;
            }
        }
            break;

        case CANCEL_PICTURE: {
            if (!error) {
                ack.type = CANCEL_PICTURE_DONE;
            } else {
                res = ERROR_CANCEL_PICTURE;
            }
        }
            break;

        case RELEASE_FRAME: {
            int index = *(int*)payload;
            client.mClFrames[index].mFrame->releaseRef();
            ack.type = RELEASE_FRAME_DONE;
        }
            break;

        case STOP_SESSION: {
            ack.type = STOP_SESSION_DONE;
        }
            break;

        default:
            fprintf(stderr, "Unexpected command %d received from client!\n", cmd.type);
            break;
    }
    return res;
}

int  CameraServer::dispatchFrame(int camId, ICameraCommand cmd,
        ICameraFrame *frame)
{
    int i;
    int currentFd;
    std::list<ClientDescriptor *>::iterator it;
    ICameraCommandType out_command;
    ICameraCommandFrameType commandFrame;
    int err_no = 0;

    out_command.type = cmd;

    if (cmd == NEW_META_FRAME) {
        out_command.payload_size = sizeof(FaceRoi);
    } else if (frame->fd == -1) {
        out_command.payload_size = frame->size;
    } else {
        out_command.payload_size = sizeof(ICameraCommandFrameType);
    }

    // Iterate over clients
    for(it = pClDescriptors.begin(); it != pClDescriptors.end(); it++) {
        if ((*it)->mCameraId == camId) {
            if (cmd == NEW_SNAPSHOT_FRAME) {
                if((*it)->mExpPicture) {
                    (*it)->mExpPicture = false;
                } else {
                    continue;
                }
            }
            // Send frame data to client
            for (i = 0; i < (*it)->mNumFrameFd; i++) {
                if ((*it)->mClFrames[i].mFrameFd == frame->fd) {
                    break;
                }
            }
            if (cmd != NEW_META_FRAME && frame->fd != -1) {
                if (i == (*it)->mNumFrameFd) {
                    (*it)->mNumFrameFd++;
                    (*it)->mClFrames[i].mFrameFd = frame->fd;
                    currentFd = frame->fd;
                } else {
                    currentFd = 0;
                }
                (*it)->mClFrames[i].mFrame = frame;
                (*it)->mClFrames[i].mFrame->acquireRef();
            }
            pthread_mutex_lock(&(*it)->mClientLock);
            socket_sendmsg((*it)->mClientFD, &out_command, sizeof(out_command), 0, &err_no);

            if (cmd == NEW_META_FRAME) {
                socket_sendmsg((*it)->mClientFD, frame->facedata,
                        sizeof(FaceRoi), 0, &err_no);
            } else if (frame->fd != -1) {
                commandFrame.index = i;
                commandFrame.bufSize = frame->size;
                commandFrame.timestamp = frame->timeStamp;
                socket_sendmsg((*it)->mClientFD, &commandFrame,
                        sizeof(ICameraCommandFrameType), currentFd, &err_no);
            } else {
                int sendBytes, totalBytes = 0;
                do {
                    sendBytes = socket_sendmsg((*it)->mClientFD, ((char*)frame->data) + totalBytes,
                            frame->size - totalBytes, 0, &err_no);
                    totalBytes += sendBytes;
                } while (sendBytes > 0 && totalBytes < frame->size);
            }
            pthread_mutex_unlock(&(*it)->mClientLock);
        }
    }
    return XNO_ERROR;
}
void CameraServer::delClient(std::list<ClientDescriptor *>::iterator it)
{
    void *cliDesc;
    // Stop and close if active
    if ((*it)->mState == CLIENT_PREVIEW) {
        stopPreview((*it)->mCameraId);
        (*it)->mState = CLIENT_OPEN;
    }
    if ((*it)->mState == CLIENT_OPEN) {
        closeCamera((*it)->mCameraId);
        (*it)->mState = CLIENT_INACTIVE;
    }
    // Report completion to server
    cliDesc = (void*)(*it);
    write(mCleanupPipeFd[1], &cliDesc, sizeof(cliDesc));
    pClDescriptors.erase(it);
    fprintf(stderr, "Active clients %d\n", pClDescriptors.size());
}

bool CameraServer::delClientByFD(int fd)
{
    bool res = false;
    std::list<ClientDescriptor *>::iterator it;
    std::list<ClientDescriptor *>::iterator itdel;
    it = pClDescriptors.begin();
    while (it != pClDescriptors.end()) {
        itdel = it;
        it++;
        if ((*itdel)->mClientFD == fd) {
            res = true;
            delClient(itdel);
            break;
        }
    }

    return res;
}

void CameraServer::delAllClients()
{
    std::list<ClientDescriptor *>::iterator it;
    std::list<ClientDescriptor *>::iterator itdel;
    int intMsg = 0;
    it = pClDescriptors.begin();
    while (it != pClDescriptors.end()) {
        itdel = it;
        it++;
        write((*itdel)->mIntCommPipeFd[1], &intMsg, sizeof(intMsg));
    }
}

static int listening = 1;
static void sigterm_handler (int sig)
{
    listening = 0;
}

int CameraServer::start()
{

    int connect_fd;
    struct sockaddr_un srv_addr, peer_addr;
    socklen_t peer_addr_size;
    pthread_t cleanupThread;
    struct sigaction sigAction;
    sigset_t stMask, oldMask;
    memset (&sigAction, 0, sizeof(sigAction));
    sigAction.sa_handler = sigterm_handler;

    if (sigaction(SIGTERM, &sigAction, 0)) {
        fprintf(stderr, "Error setting SIGTERM handler\n");
        return -1;
    }

    sigemptyset (&stMask);
    sigaddset (&stMask, SIGTERM);

    if (sigprocmask(SIG_BLOCK, &stMask, &oldMask)) {
        fprintf(stderr, "Error setting SIGTERM handler\n");
        return -1;
    }

    // Create cleanup pipe
    if(pipe(mCleanupPipeFd) < 0) {
        fprintf(stderr, "Error creating pipe\n");
        return -1;
    }

    pthread_create(&cleanupThread, NULL, cleanup_thread, (void*)&mCleanupPipeFd);

    // Create socket
    unlink(CAM_SERVER_SOCKET);
    mSocket = socket(AF_UNIX, SOCK_STREAM, 0);
    if (mSocket == -1) {
        fprintf(stderr, "Error creating socket\n");
        return -1;
    }

    memset(&srv_addr, 0, sizeof(struct sockaddr_un));
    srv_addr.sun_family = AF_UNIX;
    strncpy(srv_addr.sun_path, CAM_SERVER_SOCKET,
            sizeof(srv_addr.sun_path) - 1);

    if (bind(mSocket, (struct sockaddr *) &srv_addr,
            sizeof(struct sockaddr_un)) == -1) {
        fprintf(stderr, "Error binding socket\n");
        return -1;
    }

   if (listen(mSocket, 5) == -1) {
        fprintf(stderr, "Error while listen");
        return -1;
   }

    peer_addr_size = sizeof(struct sockaddr_un);
    while (listening) {
        fd_set fdSet;
        int readyFds;

        FD_ZERO(&fdSet);
        FD_SET(mSocket, &fdSet);

        readyFds = pselect (mSocket + 1, &fdSet, NULL, NULL, NULL, &oldMask);
        if (readyFds > 0) {
            int newFd;
            newFd = accept(mSocket, (struct sockaddr *) &peer_addr,
                         &peer_addr_size);
            if (newFd == -1) {
                fprintf(stderr, "Error accepting connection\n");
                continue;
            }

            ClientDescriptor *cliDesc = new ClientDescriptor(newFd);
            pClDescriptors.push_back(cliDesc);

            ThreadData data;
            data.client = cliDesc;
            data.server = this;

            pthread_create(&cliDesc->mThread, NULL, client_thread, (void*)&data);
        }
    }

    // Cleanup remaining clients if any
    delAllClients();

    // Stop and destroy cleanup thread
    int thread = 0;
    if (mCleanupPipeFd[1] != -1) {
        write(mCleanupPipeFd[1], &thread, sizeof(thread));
        pthread_join(cleanupThread, NULL);
        close(mCleanupPipeFd[0]);
        close(mCleanupPipeFd[1]);
    }
    if (mSocket != -1) {
        close(mSocket);
    }
    unlink(CAM_SERVER_SOCKET);

}

int main ()
{
  int res = 0;
  CameraServer server;

  res = server.start();

  return res;
}
