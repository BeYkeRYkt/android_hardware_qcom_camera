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
#include "stdio.h"
#include "CameraClient.hpp"

#define BUF_SIZE 462848

class TestCameraObject : public ICameraListener {
public:
    TestCameraObject(char *s):
            mFrameNo(0),
            mVFrameNo(0),
            mFilePrefix(s) {};                               // Constructor of Camera Object
    virtual ~TestCameraObject() {};                          // Destructor of Camera Object

    /* listener methods */
    virtual void onError()
    {
        fprintf(stderr, "Server Died!!!\n");
    };
    virtual void onPreviewFrame(ICameraFrame* frame)
    {
      char name[50];
      FILE *f;
      if(mFrameNo%10 == 0) {
        snprintf(name, 50, "/root/%s_frame_%05d.yuv", mFilePrefix, mFrameNo);
        fprintf(stderr, "Frame size %d\n", frame->size);
        f = fopen(name, "wb");
        fwrite(frame->data, frame->size, 1, f);
        fclose(f);
      }
      mFrameNo++;

      frame->releaseRef();
    };
    virtual void onVideoFrame(ICameraFrame* frame)     {
      char name[50];
      FILE *f;
      if(mVFrameNo%10 == 0) {
        snprintf(name, 50, "/root/%s_video_frame_%05d.yuv", mFilePrefix, mVFrameNo);
        fprintf(stderr, "video Frame size %d\n", frame->size);
        f = fopen(name, "wb");
        fwrite(frame->data, frame->size, 1, f);
        fclose(f);
      }
      mVFrameNo++;

      frame->releaseRef();
    };
    virtual void onPictureFrame(ICameraFrame* frame) {
        fprintf(stderr, "Picture size %d\n", frame->size);
    };
    virtual void onMetadataFrame(ICameraFrame *frame) {
        FaceRoi *faces = frame->facedata;
        fprintf(stderr, "Face %d\n", faces->number_of_faces);
    };
private:
    int mFrameNo;
    int mVFrameNo;
    char *mFilePrefix;
};

using namespace camera;

int main (int argc, char *argv[])
{
    ICameraClient *client = new ICameraClient;
    TestCameraObject *testCam = new TestCameraObject(argv[1]);
    struct CameraInfo  info;
    ImageSize preview_size, picture_size, video_size;
    int i;
    int error = 0, rc = 0;

    preview_size.width = 0;
    preview_size.height = 0;
    fprintf(stderr, "Created\n");
    client->Init();
    fprintf(stderr, "Inited\n");

    for (i = 0; i < client->GetNumCameras(); i++) {
        client->GetCameraInfo(i, &info);
        fprintf(stderr, "Camera %d func %d\n", i, info.func);
        if (info.func == CAM_FUNC_HIRES)
            break;
    }

    client->AddListener(testCam);

    rc = client->OpenCamera(i);
    fprintf(stderr, "Opened camera %d\n", i);

    rc = client->params.init(NULL);
    if (rc < 0)
        error = client->GetError();
    preview_size = client->params.getPreviewSize();
    picture_size = client->params.getPictureSize();
    video_size = client->params.getVideoSize();

    fprintf(stderr, "Default preview dimension %dx%d \n", preview_size.width, preview_size.height);
    fprintf(stderr, "Default picture dimension %dx%d \n", picture_size.width, picture_size.height);
    fprintf(stderr, "Default video dimension %dx%d \n", video_size.width, video_size.height);

    preview_size.width = 1920;
    preview_size.height = 1080;

    picture_size.width = 4208;
    picture_size.height = 3120;

    video_size.width = 1280;
    video_size.height = 720;

    client->params.setPreviewSize(preview_size);
    client->params.setPictureSize(picture_size);
    client->params.setVideoSize(video_size);

    rc = client->params.commit();
    if (rc < 0)
        error = client->GetError();
    fprintf(stderr, "Settings committed\n");
    preview_size = client->params.getPreviewSize();
    picture_size = client->params.getPictureSize();

    fprintf(stderr, "Set preview dimension %dx%d \n", preview_size.width, preview_size.height);
    fprintf(stderr, "Set picture dimension %dx%d \n", picture_size.width, picture_size.height);

    client->sendFaceDetectCommand(true);

    rc = client->StartPreview();
    if (rc < 0)
        error = client->GetError();
    fprintf(stderr, "Preview started\n");

    sleep(5);
    client->TakePicture();
    sleep(5);

    fprintf(stderr, "wait complete\n");
    rc = client->StopPreview();
    if (rc < 0)
        error = client->GetError();
    fprintf(stderr, "Preview stop\n");

    rc = client->StartRecording();
    if (rc < 0)
        error = client->GetError();
    fprintf(stderr, "Start Recording\n");

    sleep(5);
    rc = client->StopRecording();
    if (rc < 0)
        error = client->GetError();
    fprintf(stderr, "Stop Recording\n");

    rc = client->CloseCamera(i);
    if (rc < 0)
        error = client->GetError();
    fprintf(stderr, "Closed\n");

    rc = client->deInit();
    if (rc < 0)
        error = client->GetError();
    fprintf(stderr, "deInited\n");

    delete(client);

    fprintf(stderr, "Complete\n");
    return 0;
}
