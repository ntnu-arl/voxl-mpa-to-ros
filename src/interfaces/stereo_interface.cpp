/*******************************************************************************
 * Copyright 2021 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
#include <modal_pipe.h>
#include "stereo_interface.h"
#include "camera_helpers.h"

static void _frame_cb(
    __attribute__((unused)) int ch,
                            camera_image_metadata_t meta, 
                            char* frame, 
                            void* context);

StereoInterface::StereoInterface(
    ros::NodeHandle rosNodeHandle,
    int             baseChannel,
    const char *    camName) :
    GenericInterface(rosNodeHandle, baseChannel, NUM_STEREO_REQUIRED_CHANNELS, camName)
{

    char frameName[64];

    sprintf(frameName, "%s/left", m_pipeName);
    m_imageMsgL = new sensor_msgs::Image;
    m_imageMsgL->header.frame_id = frameName;
    m_imageMsgL->is_bigendian    = false;

    sprintf(frameName, "%s/right", m_pipeName);
    m_imageMsgR = new sensor_msgs::Image;
    m_imageMsgR->header.frame_id = frameName;
    m_imageMsgR->is_bigendian    = false;

    pipe_client_set_camera_helper_cb(m_baseChannel, _frame_cb, this);

}

void StereoInterface::AdvertiseTopics(){

    image_transport::ImageTransport it(m_rosNodeHandle);

    char topicName[64];

    sprintf(topicName, "/%s/left/image_raw", m_pipeName);
    m_rosImagePublisherL = it.advertise(topicName, 1);

    sprintf(topicName, "/%s/right/image_raw", m_pipeName);
    m_rosImagePublisherR = it.advertise(topicName, 1);

    m_state = ST_AD;

}
void StereoInterface::StartPublishing(){

    char fullName[MODAL_PIPE_MAX_PATH_LEN];
    pipe_client_construct_full_path(m_pipeName, fullName);

    if(pipe_client_init_channel(m_baseChannel, fullName, PIPE_CLIENT_NAME,
                EN_PIPE_CLIENT_CAMERA_HELPER, 0)){
        printf("Error opening pipe: %s\n", m_pipeName);
    } else {
        m_state = ST_RUNNING;
    }

}
void StereoInterface::StopPublishing(){

    pipe_client_close_channel(m_baseChannel);
    m_state = ST_AD;

}

void StereoInterface::CleanAndExit(){

    m_rosImagePublisherL.shutdown();
    m_rosImagePublisherR.shutdown();

    delete m_imageMsgL;
    delete m_imageMsgR;

    m_state = ST_CLEAN;

}

int StereoInterface::GetNumClients(){
    return m_rosImagePublisherL.getNumSubscribers() + m_rosImagePublisherR.getNumSubscribers();
}

// IR helper callback whenever a frame arrives
static void _frame_cb(
    __attribute__((unused)) int ch,
                            camera_image_metadata_t meta, 
                            char* frame, 
                            void* context)
{

    StereoInterface *interface = (StereoInterface *) context;

    if(meta.format != IMAGE_FORMAT_STEREO_RAW8){
        printf("Stereo interface received non-stereo frame, exiting stereo\n");
        interface->StopPublishing();
        interface->CleanAndExit();
    }

    image_transport::Publisher publisherL = interface->GetPublisherL();
    sensor_msgs::Image* imgL = interface->GetImageMsgL();

    image_transport::Publisher publisherR = interface->GetPublisherR();
    sensor_msgs::Image* imgR = interface->GetImageMsgR();

    imgL->header.stamp.fromNSec(meta.timestamp_ns);
    imgL->width    = meta.width;
    imgL->height   = meta.height;
    imgL->step     = meta.width;
    imgL->encoding = GetRosFormat(meta.format);

    imgR->header.stamp.fromNSec(meta.timestamp_ns);
    imgR->width    = meta.width;
    imgR->height   = meta.height;
    imgR->step     = meta.width;
    imgR->encoding = GetRosFormat(meta.format);

    int dataSize = imgL->step * imgL->height;

    imgL->data.resize(dataSize);
    imgR->data.resize(dataSize);

    memcpy(&(imgL->data[0]), frame, dataSize);
    memcpy(&(imgR->data[0]), &frame[dataSize], dataSize);

    publisherL.publish(*imgL);
    publisherR.publish(*imgR);
}
