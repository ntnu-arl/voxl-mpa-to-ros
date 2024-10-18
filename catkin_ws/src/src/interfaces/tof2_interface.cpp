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
#include "tof2_interface.h"

static void _helper_cb(
__attribute__((unused))    int ch, 
                           char* data, 
                           int bytes,
                           void* context); // TODO

TOF2Interface::TOF2Interface(
    ros::NodeHandle rosNodeHandle,
    ros::NodeHandle rosNodeHandleParams,
    const char *    name) :
    GenericInterface(rosNodeHandle, rosNodeHandleParams, name)
{

    //TODO Different frames
    m_pcMsg.header.frame_id = "tof";

    m_pcMsg.is_bigendian = false;
    m_pcMsg.is_dense     = true;

    pipe_client_set_simple_helper_cb(m_channel, _helper_cb, this);


    if(pipe_client_open(m_channel, name, PIPE_CLIENT_NAME,
                EN_PIPE_CLIENT_SIMPLE_HELPER | CLIENT_FLAG_START_PAUSED,
                TOF2_RECOMMENDED_READ_BUF_SIZE)){
        pipe_client_close(m_channel);//Make sure we unclaim the channel
        throw -1;
    }
}

void TOF2Interface::AdvertiseTopics(){

    char topicName[64];
    snprintf(topicName, 64, "%s", m_pipeName);
    m_pcPublisher         = m_rosNodeHandle.advertise<sensor_msgs::PointCloud2>
                                (topicName, 3);

    m_state = ST_AD;

}

void TOF2Interface::StopAdvertising(){

    m_pcPublisher.shutdown();

    m_state = ST_CLEAN;

}

int TOF2Interface::GetNumClients(){
    return m_pcPublisher.getNumSubscribers();
}

// called when the simple helper has data for us
static void _helper_cb (__attribute__((unused))int ch, char* data, int bytes, void* context)
{
    // validate that the data makes sense
    int n_packets;
    tof2_data_t* data_array = pipe_validate_tof2_data_t(data, bytes, &n_packets);

    if(data_array == NULL)
    {
      return;
    }

    TOF2Interface *interface = (TOF2Interface *) context;

    if(interface->GetState() != ST_RUNNING) return;

    sensor_msgs::PointCloud2&   pcMsg =                 interface->GetPCMsg();
    ros::Publisher&             pcPublisher =           interface->GetPCPublisher();

    // The total size of the channels of the point cloud for a single point are:
    // sizeof(float) * 4 = x, y, z, intensity
    pcMsg.point_step = (sizeof(float) * 3 + sizeof(uint8_t) * 2);

    // Describe the fields (channels) associated with each point.
    pcMsg.fields.resize(5);
    pcMsg.fields[0].name = "x";
    pcMsg.fields[1].name = "y";
    pcMsg.fields[2].name = "z";
    pcMsg.fields[3].name = "intensity";
    pcMsg.fields[4].name = "confidence";

    // Defines the format of channels.
    int index;

    for (index = 0; index < 3; index++)
    {
      pcMsg.fields[index].offset = sizeof(float) * index;
      pcMsg.fields[index].datatype = sensor_msgs::PointField::FLOAT32;
      pcMsg.fields[index].count = 1;
    }

    for (index = 3; index < 5; index++)
    {
      pcMsg.fields[index].offset = sizeof(float) * 3 + sizeof(uint8_t) * (index - 3);
      pcMsg.fields[index].datatype = sensor_msgs::PointField::UINT8;
      pcMsg.fields[index].count = 1;
    }

    // publish all the samples
    for (int i = 0; i < n_packets; i++)
    {
      pcMsg.height = data_array[i].height;
      pcMsg.width = data_array[i].width;

      pcMsg.row_step = pcMsg.point_step * pcMsg.width;
      pcMsg.data.resize(pcMsg.height * pcMsg.row_step);
      pcMsg.header.stamp = (_clock_monotonic_to_ros_time(data_array[i].timestamp_ns));

      for (uint32_t j = 0; j < pcMsg.width * pcMsg.height; j++)
      {
        *((float *)&(pcMsg.data[(j * pcMsg.point_step) + pcMsg.fields[0].offset])) = data_array[i].points[j][0];
        *((float *)&(pcMsg.data[(j * pcMsg.point_step) + pcMsg.fields[1].offset])) = data_array[i].points[j][1];
        *((float *)&(pcMsg.data[(j * pcMsg.point_step) + pcMsg.fields[2].offset])) = data_array[i].points[j][2];
        *((uint8_t *)&(pcMsg.data[(j * pcMsg.point_step) + pcMsg.fields[3].offset])) = data_array[i].grayValues[j];
        *((uint8_t *)&(pcMsg.data[(j * pcMsg.point_step) + pcMsg.fields[4].offset])) = data_array[i].confidences[j];
      }

      pcPublisher.publish(pcMsg);
    }

    return;
}
