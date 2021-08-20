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

#define NUM_PC_CHANNELS 6

static void _helper_cb(
    __attribute__((unused))int ch, 
                           char* data, 
                           int bytes, 
                           void* context);

TofInterface::TofInterface(
    ros::NodeHandle rosNodeHandle,
    ros::NodeHandle rosNodeHandleParams,
    int             baseChannel,
    const char *    camName) :
    GenericInterface(rosNodeHandle, rosNodeHandleParams, baseChannel, NUM_TOF_REQUIRED_CHANNELS, camName)
{
    int format;

    m_rosNodeHandleParams.param<int>(("tof_cutoff"), m_pcConfThreshold, 0);
    m_rosNodeHandleParams.param<std::string>(("tof_frameid"), m_frameid, "tof");

    format = IMAGE_FORMAT_RAW8;
    m_irImageMsg.header.frame_id = m_pipeName;
    m_irImageMsg.is_bigendian    = false;
    m_irImageMsg.encoding        = GetRosFormat(format);
    m_irImageMsg.width           = MPA_TOF_WIDTH;
    m_irImageMsg.height          = MPA_TOF_HEIGHT;
    m_irImageMsg.step            = MPA_TOF_WIDTH * GetStepSize(format);
    m_irImageMsg.data.resize(m_irImageMsg.step * m_irImageMsg.height);
    
    format = IMAGE_FORMAT_FLOAT32;
    m_depthImageMsg.header.frame_id = m_pipeName;
    m_depthImageMsg.is_bigendian    = false;
    m_depthImageMsg.encoding        = GetRosFormat(format);
    m_depthImageMsg.width           = MPA_TOF_WIDTH;
    m_depthImageMsg.height          = MPA_TOF_HEIGHT;
    m_depthImageMsg.step            = MPA_TOF_WIDTH * GetStepSize(format);
    m_depthImageMsg.data.resize(m_depthImageMsg.step * m_depthImageMsg.height);

    format = IMAGE_FORMAT_RAW8;
    m_confImageMsg.header.frame_id = m_pipeName;
    m_confImageMsg.is_bigendian    = false;
    m_confImageMsg.encoding        = GetRosFormat(format);
    m_confImageMsg.width           = MPA_TOF_WIDTH;
    m_confImageMsg.height          = MPA_TOF_HEIGHT;
    m_confImageMsg.step            = MPA_TOF_WIDTH * GetStepSize(format);
    m_confImageMsg.data.resize(m_confImageMsg.step * m_confImageMsg.height);

    format = IMAGE_FORMAT_FLOAT32;
    m_noiseImageMsg.header.frame_id = m_pipeName;
    m_noiseImageMsg.is_bigendian    = false;
    m_noiseImageMsg.encoding        = GetRosFormat(format);
    m_noiseImageMsg.width           = MPA_TOF_WIDTH;
    m_noiseImageMsg.height          = MPA_TOF_HEIGHT;
    m_noiseImageMsg.step            = MPA_TOF_WIDTH * GetStepSize(format);
    m_noiseImageMsg.data.resize(m_noiseImageMsg.step * m_noiseImageMsg.height);

    // these two are a little more complicated so they're in their own functions
    // for cleanliness
    InitializePointCloudMessage(m_frameid.c_str());

    pipe_client_set_simple_helper_cb(m_baseChannel, _helper_cb, this);

}

void TofInterface::AdvertiseTopics(){

    image_transport::ImageTransport it(m_rosNodeHandle);

    char topicName[64];

    sprintf(topicName, "%s/ir", m_pipeName);
    m_irImagePublisher    = it.advertiseCamera(topicName, 1);

    sprintf(topicName, "%s/depth", m_pipeName);
    m_depthImagePublisher = it.advertiseCamera(topicName, 1);

    sprintf(topicName, "%s/confidence", m_pipeName);
    m_confImagePublisher  = it.advertiseCamera(topicName, 1);

    sprintf(topicName, "%s/noise", m_pipeName);
    m_noiseImagePublisher = it.advertiseCamera(topicName, 1);

    sprintf(topicName, "%s/point_cloud", m_pipeName);
    m_pcPublisher         = m_rosNodeHandle.advertise<sensor_msgs::PointCloud2>
                                (topicName, NUM_PC_CHANNELS);

    m_state = ST_AD;

}
void TofInterface::StartPublishing(){

    char fullName[MODAL_PIPE_MAX_PATH_LEN];
    pipe_expand_location_string(m_pipeName, fullName);

    if(pipe_client_open(m_baseChannel, fullName, PIPE_CLIENT_NAME,
                EN_PIPE_CLIENT_SIMPLE_HELPER, TOF_RECOMMENDED_READ_BUF_SIZE)){
        printf("Error opening pipe: %s\n", m_pipeName);
    } else {
        pipe_client_set_disconnect_cb(m_baseChannel, _interface_dc_cb, this);
        InitializeCameraInfoMessage("map");
        m_state = ST_RUNNING;
    }

}
void TofInterface::StopPublishing(){

    pipe_client_close(m_baseChannel);
    m_state = ST_AD;

}

void TofInterface::Clean(){

    m_irImagePublisher.shutdown();

    m_depthImagePublisher.shutdown();

    m_confImagePublisher.shutdown();

    m_noiseImagePublisher.shutdown();

    m_pcPublisher.shutdown();

    m_state = ST_CLEAN;

}

int TofInterface::GetNumClients(){
    return m_irImagePublisher.   getNumSubscribers() +
           m_depthImagePublisher.getNumSubscribers() +
           m_confImagePublisher. getNumSubscribers() +
           m_noiseImagePublisher.getNumSubscribers() +
           m_pcPublisher.        getNumSubscribers();
}

// -----------------------------------------------------------------------------------------------------------------------------
// Initialize camera info message
// -----------------------------------------------------------------------------------------------------------------------------
void TofInterface::InitializeCameraInfoMessage(const char *frame_id)
{

    cJSON* json = json_fetch_object(pipe_get_info_json(m_pipeName), "lens_parameters");

    if(json == NULL){
        printf("Could not find lens parameters in info file for %s\n", m_pipeName);
        return;
    }

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
        px    = TofImageWidth  - px;
        py    = TofImageHeight - py;
        tan0 *= -1.0;
        tan1 *= -1.0;
    }

    // populate cameraInfoMsg for ROS
    m_cameraInfoMsg.header.frame_id = frame_id;
    m_cameraInfoMsg.width           = MPA_TOF_WIDTH;
    m_cameraInfoMsg.height          = MPA_TOF_HEIGHT;

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

// -----------------------------------------------------------------------------------------------------------------------------
// Initialize point cloud message
// -----------------------------------------------------------------------------------------------------------------------------
void TofInterface::InitializePointCloudMessage(const char *frame_id)
{
    m_pcMsg.header.frame_id = frame_id;
    m_pcMsg.width           = MPA_TOF_WIDTH;
    m_pcMsg.height          = MPA_TOF_HEIGHT;

    m_pcMsg.is_bigendian = false;
    // Data is not "dense" since not all points are valid.
    m_pcMsg.is_dense     = false;

    // The total size of the channels of the point cloud for a single point are:
    // sizeof(float) * 3 = x, y, and z
    // sizeof(float) = noise
    // sizeof(uint32_t) = depthConfidence (only the low byte is defined, using uin32_t for alignment)
    m_pcMsg.point_step = (sizeof(float) * 3) + sizeof(float) + sizeof(uint8_t) + sizeof(uint8_t);
    m_pcMsg.row_step   = 1;

    // Describe the fields (channels) associated with each point.
    m_pcMsg.fields.resize(NUM_PC_CHANNELS);
    m_pcMsg.fields[0].name = "x";
    m_pcMsg.fields[1].name = "y";
    m_pcMsg.fields[2].name = "z";
    m_pcMsg.fields[3].name = "noise";
    m_pcMsg.fields[4].name = "confidence";
    m_pcMsg.fields[5].name = "intensity";

    // Defines the format of x, y, z, and noise channels.
    int index;

    for (index = 0; index < 4; index++)
    {
        m_pcMsg.fields[index].offset = sizeof(float) * index;
        m_pcMsg.fields[index].datatype = sensor_msgs::PointField::FLOAT32;
        m_pcMsg.fields[index].count = 1;
    }

    // Defines the format of the depthConfidence channel.
    m_pcMsg.fields[4].offset   = m_pcMsg.fields[3].offset + sizeof(float);
    m_pcMsg.fields[4].datatype = sensor_msgs::PointField::UINT8;
    m_pcMsg.fields[4].count    = 1;

    m_pcMsg.fields[5].offset   = m_pcMsg.fields[4].offset + sizeof(uint8_t);
    m_pcMsg.fields[5].datatype = sensor_msgs::PointField::UINT8;
    m_pcMsg.fields[5].count    = 1;

    m_pcMsg.row_step = m_pcMsg.point_step * m_pcMsg.width;
    m_pcMsg.data.resize(m_pcMsg.height * m_pcMsg.row_step);

}


// called when the simple helper has data for us
static void _helper_cb(__attribute__((unused))int ch, char* data, int bytes, void* context)
{

    // validate that the data makes sense
    int n_packets;
    tof_data_t* data_array = pipe_validate_tof_data_t(data, bytes, &n_packets);
    if(data_array == NULL) return;

    TofInterface *interface = (TofInterface *) context;

    if(interface->GetState() != ST_RUNNING) return;

    sensor_msgs::CameraInfo&          cameraInfoMsg =         interface->GetCamInfo();

    sensor_msgs::Image&               irImageMsg =            interface->GetIRMsg();
    image_transport::CameraPublisher& irImagePublisher =      interface->GetIRPublisher();

    sensor_msgs::Image&               depthImageMsg =         interface->GetDepthMsg();
    image_transport::CameraPublisher& depthImagePublisher =   interface->GetDepthPublisher();

    sensor_msgs::Image&               confImageMsg =          interface->GetConfMsg();
    image_transport::CameraPublisher& confImagePublisher =    interface->GetConfPublisher();

    sensor_msgs::Image&               noiseImageMsg =         interface->GetNoiseMsg();
    image_transport::CameraPublisher& noiseImagePublisher =   interface->GetNoisePublisher();

    sensor_msgs::PointCloud2&         pcMsg =                 interface->GetPCMsg();
    ros::Publisher&                   pcPublisher =           interface->GetPCPublisher();

    int cutoff = interface->m_pcConfThreshold;

    for(int i=0;i<n_packets;i++){

        tof_data_t data = data_array[i];

        int64_t timestamp = data.timestamp_ns;
        irImageMsg    .header.stamp.fromNSec(timestamp);
        depthImageMsg .header.stamp.fromNSec(timestamp);
        confImageMsg  .header.stamp.fromNSec(timestamp);
        noiseImageMsg .header.stamp.fromNSec(timestamp);
        pcMsg         .header.stamp.fromNSec(timestamp);

        memcpy(&(irImageMsg   .data[0]), &(data.grayValues[0]),  irImageMsg.step    * irImageMsg.height);
        memcpy(&(confImageMsg .data[0]), &(data.confidences[0]), confImageMsg.step  * confImageMsg.height);
        memcpy(&(noiseImageMsg.data[0]), &(data.noises[0]),      noiseImageMsg.step * noiseImageMsg.height);

        for(int i = 0; i < MPA_TOF_SIZE; i++){

            *((float *)&(depthImageMsg.data[i * sizeof(float)])) = data.points[i][2];

            if(data.confidences[i] > cutoff){
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[0].offset])) = data.points     [i][0];
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[1].offset])) = data.points     [i][1];
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[2].offset])) = data.points     [i][2];
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[3].offset])) = data.noises     [i];
                pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[4].offset] = data.confidences[i];
                pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[5].offset] = data.grayValues [i];
            }

        }

        irImagePublisher   .publish(irImageMsg,    cameraInfoMsg);
        confImagePublisher .publish(confImageMsg,  cameraInfoMsg);
        noiseImagePublisher.publish(noiseImageMsg, cameraInfoMsg);

        depthImagePublisher.publish(depthImageMsg, cameraInfoMsg);
        pcPublisher        .publish(pcMsg);

    }

    return;
}