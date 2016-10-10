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
#include "camera.h"
#include "CameraServer.hpp"
#include "CameraClientUtil.hpp"
#include "CameraClient.hpp"
#include <algorithm>

#define CAM_SERVER_SOCKET "/root/cam_srv_sock"

#define XNO_ERROR 0 //REPLACE
using namespace camera;


CameraObject::CameraObject()
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

SrvCameraObject::SrvCameraObject(CameraServer *srv, int camId)
{
    mCamId = camId;
    mServer = srv;
    mOpenInstances = 0;
    return;
}

SrvCameraObject::~SrvCameraObject()
{
    return;
}


void SrvCameraObject::onError()
{
    return;
}

void SrvCameraObject::onPreviewFrame(ICameraFrame* frame)
{
    mServer->dispatchFrame(mCamId, NEW_PREVIEW_FRAME, frame);
    return;
}

void SrvCameraObject::onVideoFrame(ICameraFrame* frame)
{
    mServer->dispatchFrame(mCamId, NEW_VIDEO_FRAME, frame);
    return;
}

void SrvCameraObject::onPictureFrame(ICameraFrame* frame)
{
    mServer->dispatchFrame(mCamId, NEW_SNAPSHOT_FRAME, frame);
    return;
}

void SrvCameraObject::onMetadataFrame(ICameraFrame *frame)
{
    mServer->dispatchFrame(mCamId, NEW_META_FRAME, frame);
    return;
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
            fprintf(stderr, "destroying thread %p\n", &cliDesc->mThread);
            pthread_join(cliDesc->mThread, NULL);

            pthread_mutex_destroy(&cliDesc->mClientLock);
            delete(cliDesc);
            fprintf(stderr, "destroyed \n");
        } else {
            activeFlag = 0;
        }
    }
    return NULL;
}

void *listen_thread(void *arg)
{
    ThreadData data = *(ThreadData*)arg;
    bool activeFlag = true;
    ICameraCommandType inc_command;
    ICameraCommandType out_command;
    void *payload = NULL;
    void *out_payload = NULL;
    int fd = 0, error = XNO_ERROR;

    while (activeFlag) {
        // Receive cmd msg from client
        socket_recvmsg(data.client->mClientFD, &inc_command,
                sizeof(inc_command), &fd);
        if (inc_command.payload_size) {
            payload = malloc(inc_command.payload_size);
            socket_recvmsg(data.client->mClientFD, payload,
                    inc_command.payload_size, &fd);
        }
        // process command
        error = data.server->processCommand(*data.client, inc_command, payload,
                out_command, &out_payload);
        if (error == XNO_ERROR && out_command.type == STOP_SESSION_DONE) {
            // We just stopped camera. Stop listening and proceed to cleanup
            activeFlag = false;
        }

        // send acknowledge
        pthread_mutex_lock(&data.client->mClientLock);
        socket_sendmsg(data.client->mClientFD, &out_command,
                sizeof(out_command), fd);
        if (out_command.payload_size) {
            socket_sendmsg(data.client->mClientFD, out_payload,
                    out_command.payload_size, fd);
        }
        pthread_mutex_unlock(&data.client->mClientLock);
        if (payload) {
            free(payload);
            payload = NULL;
        }
        if (out_payload) {
            free(out_payload);
            out_payload = NULL;
        }

    }

    data.server->delClientByFD(data.client->mClientFD);

    return NULL;
}

CameraServer::CameraServer ()
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
    error = pCamObjects[camId]->mCamera->startPreview();
    return error;
}

int  CameraServer::stopPreview(int camId)
{
    int error;
    pCamObjects[camId]->mCamera->stopPreview();
    return error;
}

int  CameraServer::startRecording(int camId)
{
    int error;
    error = pCamObjects[camId]->mCamera->startRecording();
    return error;
}

int  CameraServer::stopRecording(int camId)
{
    int error;
    pCamObjects[camId]->mCamera->stopRecording();
    return error;
}

int  CameraServer::closeCamera(int camId)
{


    pCamObjects[camId]->mOpenInstances--;
    if (pCamObjects[camId]->mOpenInstances == 0) {
        pCamObjects[camId]->mCamera->removeListener(pCamObjects[camId]);
        ICameraDevice::deleteInstance(&pCamObjects[camId]->mCamera);
        delete (pCamObjects[camId]);
        pCamObjects[camId] = NULL;
    }
    return XNO_ERROR;
}

int  CameraServer::getParameters(int camId)
{
    return XNO_ERROR;
}

int  CameraServer::setParameters(int camId)
{
    return XNO_ERROR;
}

int  CameraServer::takePicture(int camId)
{
    int error;
    error = pCamObjects[camId]->mCamera->takePicture();
    return XNO_ERROR;
}

int  CameraServer::cancelPicture(int camId)
{
    return XNO_ERROR;
}

int  CameraServer::enableFaceDetect(int camId, bool enable)
{
    pCamObjects[camId]->mCamera->sendFaceDetectCommand(enable);
    return XNO_ERROR;
}

int  CameraServer::processCommand(ClientDescriptor &client,
        const ICameraCommandType &cmd, void* payload,
        ICameraCommandType &ack, void **ack_payload)
{
    int error = XNO_ERROR;
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
            openCamera(client.mCameraId, client.mMaster);
            ack.type = OPEN_CAMERA_DONE;
            ack.payload_size = sizeof(int);
            *ack_payload = malloc(ack.payload_size);
            isMaster = (int*)(*ack_payload);
            *isMaster = client.mMaster;
        }
            break;

        case CLOSE_CAMERA: {
            closeCamera(client.mCameraId);
            ack.type = CLOSE_CAMERA_DONE;
        }
            break;

        case GET_PARAMETERS: {
            // TBD
            ack.type = GET_PARAMETERS_DONE;
        }
            break;

        case SET_PARAMETERS: {
            // TBD
            ack.type = SET_PARAMETERS_DONE;
        }
            break;

        case ENABLE_FACE_DETECT: {
            bool enable = false;
            // extract enable from command
            enableFaceDetect(client.mCameraId, enable);
            ack.type = ENABLE_FACE_DETECT_DONE;
        }
            break;

        case START_PREVIEW: {
            error = startPreview(client.mCameraId);
            ack.type = START_PREVIEW_DONE;
        }
            break;

        case STOP_PREVIEW: {
            error = stopPreview(client.mCameraId);
            ack.type = STOP_PREVIEW_DONE;
        }
            break;

        case START_RECORDING: {
            error = startRecording(client.mCameraId);
            ack.type = START_RECORDING_DONE;
        }
            break;

        case STOP_RECORDING: {
            error = stopRecording(client.mCameraId);
            ack.type = STOP_PRECORDING_DONE;
        }
            break;

        case TAKE_PICTURE: {
            takePicture(client.mCameraId);
            ack.type = TAKE_PICTURE_DONE;
        }
            break;

        case CANCEL_PICTURE: {
            // TBD
            ack.type = CANCEL_PICTURE_DONE;
        }
            break;

        case STOP_SESSION: {
            ack.type = STOP_SESSION_DONE;
        }
            break;

        default:
            // Unexpected command Report error.
            break;
    }
    return error;
}

int  CameraServer::dispatchFrame(int camId, ICameraCommand cmd,
        const ICameraFrame *frame)
{
    int i;
    int currentFd;
    std::list<ClientDescriptor *>::iterator it;
    ICameraCommandType out_command;
    ICameraCommandFrameType commandFrame;

    out_command.type = cmd;
    out_command.payload_size = sizeof(ICameraCommandFrameType);

    // Iterate over clients
    for(it = pClDescriptors.begin(); it != pClDescriptors.end(); it++) {
      /*fprintf(stderr, "camid %d Descriptor %d frame %d\n", (*it)->mCameraId, (*it)->mClientFD, frame->fd);*/
        if ((*it)->mCameraId == camId) {
            // Send frame data to client
            for (i = 0; i < (*it)->mNumFrameFd; i++) {
                if ((*it)->mFrameFds[i] == frame->fd) {
                    break;
                }
            }

            pthread_mutex_lock(&(*it)->mClientLock);
            socket_sendmsg((*it)->mClientFD, &out_command, sizeof(out_command), 0);
            if (i == (*it)->mNumFrameFd) {
                (*it)->mNumFrameFd++;
                (*it)->mFrameFds[i] = frame->fd;
                currentFd = frame->fd;
            } else {
                currentFd = 0;
            }
            commandFrame.index = i;
            commandFrame.bufSize = frame->size;
            commandFrame.timestamp = frame->timeStamp;
            socket_sendmsg((*it)->mClientFD, &commandFrame,
                    sizeof(ICameraCommandFrameType), currentFd);
            pthread_mutex_unlock(&(*it)->mClientLock);
        }
    }
    return XNO_ERROR;
}

bool CameraServer::delClientByFD(int fd)
{
    std::list<ClientDescriptor *>::iterator it;
    it = pClDescriptors.begin();
    void *cliDesc;
    while (it != pClDescriptors.end()) {
        if ((*it)->mClientFD == fd) {
    // Report completion to server
        fprintf(stderr, "found %d\n", fd);
            cliDesc = (void*)(*it);
            write(mCleanupPipeFd[1], &cliDesc, sizeof(cliDesc));
            break;
        }
        it++;
    }

    if (it != pClDescriptors.end())
        pClDescriptors.erase(it);

    return (it != pClDescriptors.end());
}

void CameraServer::start()
{

    int connect_fd;
    struct sockaddr_un srv_addr, peer_addr;
    socklen_t peer_addr_size;
    bool listening = true;
    pthread_t cleanupThread;

    // Create cleanup pipe
    if(pipe(mCleanupPipeFd) < 0) {
        fprintf(stderr, "Error creating pipe\n");
        // TODO HANDLE ERROR
    }

    pthread_create(&cleanupThread, NULL, cleanup_thread, (void*)&mCleanupPipeFd);

    // Create socket 
    unlink(CAM_SERVER_SOCKET);
    mSocket = socket(AF_UNIX, SOCK_STREAM, 0);
    if (mSocket == -1) {
        fprintf(stderr, "Error creating socket\n");
        // TODO HANDLE ERROR
    }

    memset(&srv_addr, 0, sizeof(struct sockaddr_un));
    srv_addr.sun_family = AF_UNIX;
    strncpy(srv_addr.sun_path, CAM_SERVER_SOCKET,
            sizeof(srv_addr.sun_path) - 1);

    if (bind(mSocket, (struct sockaddr *) &srv_addr,
            sizeof(struct sockaddr_un)) == -1) {
        fprintf(stderr, "Error binding socket\n");
        // TODO HANDLE ERROR
    }

   if (listen(mSocket, 5) == -1)
        fprintf(stderr, "Error while listen");

    peer_addr_size = sizeof(struct sockaddr_un);
    while (listening) {
        ClientDescriptor *cliDesc = new ClientDescriptor;
        cliDesc->mNumFrameFd = 0;
        ThreadData data;
        pClDescriptors.push_back(cliDesc);
        cliDesc->mClientFD = accept(mSocket, (struct sockaddr *) &peer_addr,
                     &peer_addr_size);

        pthread_mutex_init(&cliDesc->mClientLock, NULL);
        fprintf(stderr, "Connect %d\n", cliDesc->mClientFD );
        data.client = cliDesc;
        data.server = this;
        if (cliDesc->mClientFD == -1)
            fprintf(stderr, "Error accepting connection\n");
        pthread_create(&cliDesc->mThread, NULL, listen_thread, (void*)&data);
        fprintf(stderr, "creating thread %p\n", &cliDesc->mThread);
    }

    // Check if all cameras are closed

    // Stop and destroy cleanup thread
    void *thread = NULL;
    write(mCleanupPipeFd[1], thread, sizeof(thread));
    pthread_join(cleanupThread, NULL);
    close(mCleanupPipeFd[0]);
    close(mCleanupPipeFd[1]);
}

int main ()
{
  CameraServer server;

  server.start();

  return 0;
}
