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
#include "point_cloud_interface.h"
#include "camera_helpers.h"

static void _helper_cb(
    __attribute__((unused))int ch, 
                           point_cloud_metadata_t meta,
                           void* data, 
                           void* context);

PointCloudInterface::PointCloudInterface(
    ros::NodeHandle rosNodeHandle,
    ros::NodeHandle rosNodeHandleParams,
    int             baseChannel,
    const char *    camName) :
    GenericInterface(rosNodeHandle, rosNodeHandleParams, baseChannel, NUM_TOF_REQUIRED_CHANNELS, camName)
{
    cJSON* info = pipe_get_info_json(m_pipeName);

    json_fetch_int_with_default(info, "pc_format",  &m_inputPCType, POINT_CLOUD_FORMAT_FLOAT_XYZ);

    //TODO Different frames
    m_pcMsg.header.frame_id = "world";

    m_pcMsg.is_bigendian = false;
    // Data is not "dense" since not all points are valid.
    m_pcMsg.is_dense     = false;

    // The total size of the channels of the point cloud for a single point are:
    // sizeof(float) * 3 = x, y, and z
    // sizeof(float) = noise
    // sizeof(uint32_t) = depthConfidence (only the low byte is defined, using uin32_t for alignment)
    m_pcMsg.point_step = (sizeof(float) * 3);
    m_pcMsg.row_step   = 1;

    // Describe the fields (channels) associated with each point.
    m_pcMsg.fields.resize(3);
    m_pcMsg.fields[0].name = "x";
    m_pcMsg.fields[1].name = "y";
    m_pcMsg.fields[2].name = "z";

    // Defines the format of x, y, z, and noise channels.
    int index;

    for (index = 0; index < 3; index++)
    {
        m_pcMsg.fields[index].offset = sizeof(float) * index;
        m_pcMsg.fields[index].datatype = sensor_msgs::PointField::FLOAT32;
        m_pcMsg.fields[index].count = 1;
    }

    pipe_client_set_point_cloud_helper_cb(m_baseChannel, _helper_cb, this);

}

void PointCloudInterface::AdvertiseTopics(){

    char topicName[64];
    sprintf(topicName, "%s", m_pipeName);
    m_pcPublisher         = m_rosNodeHandle.advertise<sensor_msgs::PointCloud2>
                                (topicName, 3);

    m_state = ST_AD;

}
void PointCloudInterface::StartPublishing(){

    char fullName[MODAL_PIPE_MAX_PATH_LEN];
    pipe_expand_location_string(m_pipeName, fullName);

    if(pipe_client_open(m_baseChannel, fullName, PIPE_CLIENT_NAME,
                CLIENT_FLAG_EN_POINT_CLOUD_HELPER, TOF_RECOMMENDED_READ_BUF_SIZE)){
        printf("Error opening pipe: %s\n", m_pipeName);
    } else {
        pipe_client_set_disconnect_cb(m_baseChannel, _interface_dc_cb, this);
        m_state = ST_RUNNING;
    }

}
void PointCloudInterface::StopPublishing(){

    pipe_client_close(m_baseChannel);
    m_state = ST_AD;

}

void PointCloudInterface::Clean(){

    m_pcPublisher.shutdown();

    m_state = ST_CLEAN;

}

int PointCloudInterface::GetNumClients(){
    return m_pcPublisher.getNumSubscribers();
}

// called when the simple helper has data for us
static void _helper_cb (int ch, point_cloud_metadata_t meta, void* data, void* context)
{

    PointCloudInterface *interface = (PointCloudInterface *) context;

    float* dataPoints = (float*)data;

    if(interface->GetState() != ST_RUNNING) return;

    sensor_msgs::PointCloud2&   pcMsg =                 interface->GetPCMsg();
    ros::Publisher&             pcPublisher =           interface->GetPCPublisher();

    if(pcMsg.height != 1 || pcMsg.width != meta.n_points){
        pcMsg.height = 1;
        pcMsg.width  = meta.n_points;

        pcMsg.row_step = pcMsg.point_step * pcMsg.width;
        pcMsg.data.resize(pcMsg.height * pcMsg.row_step);
    }

    int64_t timestamp = meta.timestamp_ns;
    pcMsg.header.stamp.fromNSec(timestamp);

    for(uint32_t i = 0; i < meta.n_points; i++){

        *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[0].offset])) = dataPoints[i*3 + 0];
        *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[1].offset])) = dataPoints[i*3 + 1];
        *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[2].offset])) = dataPoints[i*3 + 2];

    }
    pcPublisher.publish(pcMsg);

    return;
}