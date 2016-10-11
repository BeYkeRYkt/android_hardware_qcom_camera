/* Copyright (c) 2012-2014, 2016, The Linux Foundation. All rights reserved.
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
#include <errno.h>
#include "CameraClientUtil.hpp"
int socket_sendmsg (int fd, void *msg, uint32_t buf_size, int sendfd,
        int *err_no)
{
    struct msghdr msgh;
    struct iovec iov[1];
    struct cmsghdr * cmsghp = NULL;
    char control[CMSG_SPACE(sizeof(int))];
    int send_len = 0;

    *err_no = 0;
    if (msg == NULL) {
      fprintf(stderr,"%s: msg is NULL", __func__);
      return -1;
    }
    memset(&msgh, 0, sizeof(msgh));
    msgh.msg_name = NULL;
    msgh.msg_namelen = 0;

    iov[0].iov_base = msg;
    iov[0].iov_len = buf_size;
    msgh.msg_iov = iov;
    msgh.msg_iovlen = 1;
    /*fprintf(stderr,"%s: iov_len=%d\n", __func__, iov[0].iov_len);*/

    msgh.msg_control = NULL;
    msgh.msg_controllen = 0;

    /* if sendfd is valid, we need to pass it through control msg */
    if( sendfd > 0) {
      msgh.msg_control = control;
      msgh.msg_controllen = sizeof(control);
      cmsghp = CMSG_FIRSTHDR(&msgh);
      if (cmsghp != NULL) {
        /*fprintf(stderr,"%s: Got ctrl msg pointer\n", __func__);*/
        cmsghp->cmsg_level = SOL_SOCKET;
        cmsghp->cmsg_type = SCM_RIGHTS;
        cmsghp->cmsg_len = CMSG_LEN(sizeof(int));
        *((int *)CMSG_DATA(cmsghp)) = sendfd;
        /*fprintf(stderr,"%s: cmsg data=%d\n", __func__, *((int *) CMSG_DATA(cmsghp)));*/
      } else {
        fprintf(stderr,"%s: ctrl msg NULL\n", __func__);
        return -1;
      }
    }

    send_len = sendmsg(fd, &(msgh), 0);
    *err_no = errno;
    return send_len;
}

int socket_recvmsg(int fd, void *msg, uint32_t buf_size, int *rcvdfd,
        int *err_no)
{
    struct msghdr msgh;
    struct iovec iov[1];
    struct cmsghdr *cmsghp = NULL;
    char control[CMSG_SPACE(sizeof(int))];
    int rcvd_fd = -1;
    int rcvd_len = 0;

    *err_no = 0;
    if ( (msg == NULL) || (buf_size <= 0) ) {
      fprintf(stderr," %s: msg buf is NULL", __func__);
      return -1;
    }

    memset(&msgh, 0, sizeof(msgh));
    msgh.msg_name = NULL;
    msgh.msg_namelen = 0;
    msgh.msg_control = control;
    msgh.msg_controllen = sizeof(control);

    iov[0].iov_base = msg;
    iov[0].iov_len = buf_size;
    msgh.msg_iov = iov;
    msgh.msg_iovlen = 1;

    rcvd_len = recvmsg(fd, &(msgh), 0);
    *err_no = errno;
    if (rcvd_len <= 0) {
      fprintf(stderr," %s: recvmsg failed %d\n", __func__, errno);
      return rcvd_len;
    }

    /*fprintf(stderr,"%s:  msg_ctrl %p len %d\n", __func__, msgh.msg_control, msgh.msg_controllen);*/

    if( ((cmsghp = CMSG_FIRSTHDR(&msgh)) != NULL) &&
        (cmsghp->cmsg_len == CMSG_LEN(sizeof(int))) ) {
      if (cmsghp->cmsg_level == SOL_SOCKET &&
        cmsghp->cmsg_type == SCM_RIGHTS) {
        /*fprintf(stderr,"%s:  CtrlMsg is valid\n", __func__);*/
        rcvd_fd = *((int *) CMSG_DATA(cmsghp));
        /*fprintf(stderr,"%s:  Receieved fd=%d\n", __func__, rcvd_fd);*/
      } else {
        fprintf(stderr,"%s:  Unexpected Control Msg. Line=%d\n", __func__, __LINE__);
      }
    }

    *rcvdfd = rcvd_fd;

    return rcvd_len;
}


