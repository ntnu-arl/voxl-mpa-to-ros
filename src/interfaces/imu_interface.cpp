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
#include "imu_interface.h"

static void _helper_cb(
    __attribute__((unused))int ch, 
                           char* data, 
                           int bytes, 
                           void* context);

IMUInterface::IMUInterface(
    ros::NodeHandle rosNodeHandle,
    int             baseChannel,
    const char *    name) :
    GenericInterface(rosNodeHandle, baseChannel, NUM_IMU_REQUIRED_CHANNELS, name)
{

    m_imuMsg.header.frame_id = "map";
    m_imuMsg.orientation.x = 0;
    m_imuMsg.orientation.y = 0;
    m_imuMsg.orientation.z = 0;
    m_imuMsg.orientation.w = 0;
    m_imuMsg.orientation_covariance         = {-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0};
    m_imuMsg.angular_velocity_covariance    = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    m_imuMsg.linear_acceleration_covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    pipe_client_set_simple_helper_cb(m_baseChannel, _helper_cb, this);

}

void IMUInterface::AdvertiseTopics(){

    char topicName[64];

    sprintf(topicName, "/%s", m_pipeName);
    m_rosPublisher = ((ros::NodeHandle) m_rosNodeHandle).advertise<sensor_msgs::Imu>(topicName, 1);

    m_state = ST_AD;

}
void IMUInterface::StartPublishing(){

    char fullName[MODAL_PIPE_MAX_PATH_LEN];
    pipe_client_construct_full_path(m_pipeName, fullName);

    if(pipe_client_init_channel(m_baseChannel, fullName, PIPE_CLIENT_NAME,
                EN_PIPE_CLIENT_SIMPLE_HELPER, IMU_RECOMMENDED_READ_BUF_SIZE)){
        printf("Error opening pipe: %s\n", m_pipeName);
    } else {
        pipe_client_set_disconnect_cb(m_baseChannel, _interface_dc_cb, this);
        m_state = ST_RUNNING;
    }

}
void IMUInterface::StopPublishing(){

    pipe_client_close_channel(m_baseChannel);
    m_state = ST_AD;

}

void IMUInterface::Clean(){

    m_rosPublisher.shutdown();

    m_state = ST_CLEAN;

}

int IMUInterface::GetNumClients(){
    return m_rosPublisher.getNumSubscribers();
}

// called when the simple helper has data for us
static void _helper_cb(__attribute__((unused))int ch, char* data, int bytes, void* context)
{

    // validate that the data makes sense
    int n_packets;
    imu_data_t* data_array = modal_imu_validate_pipe_data(data, bytes, &n_packets);
    if(data_array == NULL) return;

    IMUInterface *interface = (IMUInterface *) context;
    ros::Publisher publisher = interface->GetPublisher();
    sensor_msgs::Imu imu = interface->GetImuMsg();

    // make a new data struct to hold the average
    imu_data_t avg;
    memset(&avg,0,sizeof(avg));

    //publish all the samples
    for(int i=0;i<n_packets;i++){

        imu.header.stamp.fromNSec(data_array[i].timestamp_ns);
        imu.angular_velocity.x = data_array[i].gyro_rad[0];
        imu.angular_velocity.y = data_array[i].gyro_rad[1];
        imu.angular_velocity.z = data_array[i].gyro_rad[2];
        imu.linear_acceleration.x = data_array[i].accl_ms2[0];
        imu.linear_acceleration.y = data_array[i].accl_ms2[1];
        imu.linear_acceleration.z = data_array[i].accl_ms2[2];

        publisher.publish(imu);

    }


    return;
}
