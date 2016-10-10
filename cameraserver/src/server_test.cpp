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
#include "../libcamera/src/camera_memory.h"
#include "stdio.h"
#include "CameraClient.hpp"

#define BUF_SIZE 462848

class TestCameraObject : public ICameraListener {
public:
    TestCameraObject(char *s)
        {
            mFileprefix = s;
            mFrameNo = 0;
        };                                   // Constructor of Camera Object
    virtual ~TestCameraObject() {};                          // Destructor of Camera Object

    /* listener methods */
    virtual void onError() {};
    virtual void onPreviewFrame(ICameraFrame* frame)
    {
      char name[50];
      FILE *f;
      if(mFrameNo%10 == 0) {
        snprintf(name, 50, "/root/%s_frame_%05d.yuv", mFileprefix, mFrameNo);
        fprintf(stderr, "Frame size %d\n", frame->size);
        f = fopen(name, "wb");
        fwrite(frame->data, frame->size, 1, f);
        fclose(f);
      }
      mFrameNo++;
    };
    virtual void onVideoFrame(ICameraFrame* frame) {};
    virtual void onPictureFrame(ICameraFrame* frame) {};
    virtual void onMetadataFrame(ICameraFrame *frame) {};
private:
    int mFrameNo;
    char *mFileprefix;
};

using namespace camera;

int main (int argc, char *argv[])
{
    ICameraClient *client = new ICameraClient;
    TestCameraObject *testCam = new TestCameraObject(argv[1]);
    struct CameraInfo  info;
    fprintf(stderr, "Created\n");
    client->Init();
    fprintf(stderr, "Inited\n");

    for (int i = 0; i < client->GetNumCameras(); i++) {
        client->GetCameraInfo(i, &info);
        fprintf(stderr, "Camera %d func %d\n", i, info.func);
    }

    client->AddListener(testCam);

    client->OpenCamera(0);
    fprintf(stderr, "Opened\n");

    client->StartPreview();
    fprintf(stderr, "Preview started\n");

    sleep(10);

    fprintf(stderr, "wait complete\n");
    client->StopPreview();
    fprintf(stderr, "Preview stop\n");

    client->CloseCamera(0);
    fprintf(stderr, "Closed\n");

    client->deInit();
    fprintf(stderr, "deInited\n");

    delete(client);

    fprintf(stderr, "Complete\n");
    return 0;
}
