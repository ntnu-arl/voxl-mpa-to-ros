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
#include <modal_json.h>
#include "tof_interface.h"
#include "camera_helpers.h"

static void _IR_frame_cb(
    __attribute__((unused)) int ch,
                            camera_image_metadata_t meta, 
                            char* frame, 
                            void* context);

TofInterface::TofInterface(
    ros::NodeHandle rosNodeHandle,
    int             baseChannel,
    const char *    camName) :
    GenericInterface(rosNodeHandle, baseChannel, NUM_TOF_REQUIRED_CHANNELS, camName)
{

    sprintf(m_baseName, "%.*s", (strlen(m_pipeName) - strlen("_pc")), m_pipeName);
    sprintf(m_irName, "%s_ir", m_baseName);

    m_irImageMsg = new sensor_msgs::Image;
    m_irImageMsg->header.frame_id = m_baseName;
    m_irImageMsg->is_bigendian    = false;

    pipe_client_set_camera_helper_cb(TOF_IR_CHANNEL, _IR_frame_cb, this);

}

void TofInterface::AdvertiseTopics(){

    image_transport::ImageTransport it(m_rosNodeHandle);

    char topicName[64];

    sprintf(topicName, "/%s", m_irName);
    m_irImagePublisher = it.advertiseCamera(topicName, 1);

    m_state = ST_AD;

}
void TofInterface::StartPublishing(){

    char fullName[MODAL_PIPE_MAX_PATH_LEN];
    pipe_client_construct_full_path(m_irName, fullName);

    if(pipe_client_init_channel(TOF_IR_CHANNEL, fullName, PIPE_CLIENT_NAME,
                EN_PIPE_CLIENT_CAMERA_HELPER, 0)){
        printf("Error opening pipe: %s\n", m_pipeName);
        return;
    }

    InitializeCameraInfoMessage();
    m_state = ST_RUNNING;

}
void TofInterface::StopPublishing(){

    pipe_client_close_channel(TOF_IR_CHANNEL);
    m_state = ST_AD;

}

void TofInterface::Clean(){

    m_irImagePublisher.shutdown();
    delete m_irImageMsg;
    m_state = ST_CLEAN;

}

int TofInterface::GetNumClients(){
    return m_irImagePublisher.getNumSubscribers();
}

// -----------------------------------------------------------------------------------------------------------------------------
// Initialize camera info message
// -----------------------------------------------------------------------------------------------------------------------------
void TofInterface::InitializeCameraInfoMessage()
{

    char infoString[512];
    if(pipe_client_get_info_string(TOF_IR_CHANNEL, infoString, 512) < 0) {
        printf("Error getting tof camera info on channel: %d!\n", TOF_IR_CHANNEL);
        return;
    }

    cJSON* json = cJSON_Parse((const char *) infoString);

    //All defaults are the current calibration values at the time of writing
    double fx;
    json_fetch_double_with_default(json, "fx", &fx, 111.08799743652344);
    double fy;
    json_fetch_double_with_default(json, "fy", &fy, 111.08799743652344);
    double rad0;
    json_fetch_double_with_default(json, "rad0", &rad0, -0.2603900134563446);
    double rad1;
    json_fetch_double_with_default(json, "rad1", &rad1, 0.095771998167037964);
    double rad2;
    json_fetch_double_with_default(json, "rad2", &rad2, -0.01594259962439537);

    double px, py, tan0, tan1;
    json_fetch_double_with_default(json, "cx", &px, 114.58200073242188);
    json_fetch_double_with_default(json, "cy", &py, 89.16400146484375);
    json_fetch_double_with_default(json, "tan0", &tan0, -0.0024587700609117746);
    json_fetch_double_with_default(json, "tan1", &tan1, -0.0027381700929254293);

    cJSON_Delete(json);

    // when flipping the image, tangential distortion must be flipped
    // and principle point reflected over the center of the image
    if(FLIPIMAGE)
    {
        px   = TofImageWidth  - px;
        py   = TofImageHeight - py;
        tan0 *= -1.0;
        tan1 *= -1.0;
    }

    // populate cameraInfoMsg for ROS
    m_cameraInfoMsg.header.frame_id = m_baseName;
    m_cameraInfoMsg.width           = 224;
    m_cameraInfoMsg.height          = 172;

    // distortion parameters. 5-param radtan model identical to factory sensor cal
    m_cameraInfoMsg.distortion_model = "plumb_bob";
    m_cameraInfoMsg.D.resize(5);
    m_cameraInfoMsg.D[0] = rad0;
    m_cameraInfoMsg.D[1] = rad1;
    m_cameraInfoMsg.D[2] = tan0;
    m_cameraInfoMsg.D[3] = tan1;
    m_cameraInfoMsg.D[4] = rad2;

    // Intrinsic camera matrix for the raw (distorted) images.
    m_cameraInfoMsg.K[0] = fx;
    m_cameraInfoMsg.K[1] = 0;
    m_cameraInfoMsg.K[2] = px;
    m_cameraInfoMsg.K[3] = 0;
    m_cameraInfoMsg.K[4] = fy;
    m_cameraInfoMsg.K[5] = py;
    m_cameraInfoMsg.K[6] = 0;
    m_cameraInfoMsg.K[7] = 0;
    m_cameraInfoMsg.K[8] = 1;

    // Rectification matrix STEREO CAM ONLY, leave as identity for monocular
    m_cameraInfoMsg.R[0] = 1;
    m_cameraInfoMsg.R[1] = 0;
    m_cameraInfoMsg.R[2] = 0;
    m_cameraInfoMsg.R[3] = 0;
    m_cameraInfoMsg.R[4] = 1;
    m_cameraInfoMsg.R[5] = 0;
    m_cameraInfoMsg.R[6] = 0;
    m_cameraInfoMsg.R[7] = 0;
    m_cameraInfoMsg.R[8] = 1;

    // Projection matrix
    m_cameraInfoMsg.P[0] = fx;
    m_cameraInfoMsg.P[1] = 0;
    m_cameraInfoMsg.P[2] = px;
    m_cameraInfoMsg.P[3] = 0;  // Tx for stereo offset
    m_cameraInfoMsg.P[4] = 0;
    m_cameraInfoMsg.P[5] = fy;
    m_cameraInfoMsg.P[6] = py;
    m_cameraInfoMsg.P[7] = 0;  // Ty for stereo offset
    m_cameraInfoMsg.P[8] = 0;
    m_cameraInfoMsg.P[9] = 0;
    m_cameraInfoMsg.P[10] = 1;
    m_cameraInfoMsg.P[11] = 0;
}

// IR helper callback whenever a frame arrives
static void _IR_frame_cb(
    __attribute__((unused)) int ch,
                            camera_image_metadata_t meta, 
                            char* frame, 
                            void* context)
{

    TofInterface *interface = (TofInterface *) context;
    image_transport::CameraPublisher publisher = interface->GetIRPublisher();
    sensor_msgs::Image* img = interface->GetIRMsg();

    img->header.stamp.fromNSec(meta.timestamp_ns);
    img->width    = meta.width;
    img->height   = meta.height;
    img->step     = meta.width * GetStepSize(meta.format);
    img->encoding = GetRosFormat(meta.format);

    int dataSize = img->step * img->height;

    img->data.resize(dataSize);

    memcpy(&(img->data[0]), frame, dataSize);

    publisher.publish(*img, interface->GetCamInfo());
}
/*
// noise helper callback whenever a frame arrives
static void _noise_frame_cb(
    __attribute__((unused)) int ch,
                            camera_image_metadata_t meta, 
                            char* frame, 
                            void* context)
{

    TofInterface *interface = (TofInterface *) context;
    image_transport::CameraPublisher publisher = interface->GetIRPublisher();
    sensor_msgs::Image* img = interface->GetIRMsg();

    img->header.stamp.fromNSec(meta.timestamp_ns);
    img->width    = meta.width;
    img->height   = meta.height;
    img->step     = meta.width * GetStepSize(meta.format);
    img->encoding = GetRosFormat(meta.format);

    int dataSize = img->step * img->height;

    img->data.resize(dataSize);

    memcpy(&(img->data[0]), frame, dataSize);

    publisher.publish(*img, interface->GetCamInfo());
}

// confidence helper callback whenever a frame arrives
static void _conf_frame_cb(
    __attribute__((unused)) int ch,
                            camera_image_metadata_t meta, 
                            char* frame, 
                            void* context)
{

    TofInterface *interface = (TofInterface *) context;
    image_transport::CameraPublisher publisher = interface->GetIRPublisher();
    sensor_msgs::Image* img = interface->GetIRMsg();

    img->header.stamp.fromNSec(meta.timestamp_ns);
    img->width    = meta.width;
    img->height   = meta.height;
    img->step     = meta.width * GetStepSize(meta.format);
    img->encoding = GetRosFormat(meta.format);

    int dataSize = img->step * img->height;

    img->data.resize(dataSize);

    memcpy(&(img->data[0]), frame, dataSize);

    publisher.publish(*img, interface->GetCamInfo());
}
*/