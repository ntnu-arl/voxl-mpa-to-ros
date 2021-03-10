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
#include "vio_interface.h"

static void _helper_cb(
    __attribute__((unused))int ch, 
                           char* data, 
                           int bytes, 
                           void* context);

VIOInterface::VIOInterface(
    ros::NodeHandle rosNodeHandle,
    int             baseChannel,
    const char *    name) :
    GenericInterface(rosNodeHandle, baseChannel, NUM_VIO_REQUIRED_CHANNELS, name)
{

    pipe_client_set_simple_helper_cb(m_baseChannel, _helper_cb, this);


}

void VIOInterface::AdvertiseTopics(){

    char frameName[64];

    sprintf(frameName, "/%s/pose", m_pipeName);

    m_posePublisher = ((ros::NodeHandle) m_rosNodeHandle).advertise<geometry_msgs::PoseStamped>(frameName,1);

    sprintf(frameName, "/%s/odometry", m_pipeName);
    m_odomPublisher = ((ros::NodeHandle) m_rosNodeHandle).advertise<nav_msgs::Odometry>(frameName,1);

    sprintf(frameName, "/%s/internal_states", m_pipeName);
    m_statePublisher = ((ros::NodeHandle) m_rosNodeHandle).advertise<voxl_mpa_to_ros::InternalStates>(frameName,1);

    m_state = ST_AD;

}
void VIOInterface::StartPublishing(){

    char fullName[MODAL_PIPE_MAX_PATH_LEN];
    pipe_client_construct_full_path(m_pipeName, fullName);

    if(pipe_client_init_channel(m_baseChannel, fullName, PIPE_CLIENT_NAME,
                EN_PIPE_CLIENT_SIMPLE_HELPER, VIO_RECOMMENDED_READ_BUF_SIZE)){
        printf("Error opening pipe: %s\n", m_pipeName);
    } else {
        m_state = ST_RUNNING;
    }

}
void VIOInterface::StopPublishing(){

    pipe_client_close_channel(m_baseChannel);
    m_state = ST_AD;

}

void VIOInterface::CleanAndExit(){

    m_posePublisher.shutdown();
    m_odomPublisher.shutdown();
    m_statePublisher.shutdown();
    delete m_poseMsg;
    delete m_odomMsg;
    delete m_stateMsg;
    m_state = ST_CLEAN;

}

int VIOInterface::GetNumClients(){
    return m_posePublisher.getNumSubscribers() + m_odomPublisher.getNumSubscribers() + m_statePublisher.getNumSubscribers();
}

// called when the simple helper has data for us
static void _helper_cb(__attribute__((unused))int ch, char* data, int bytes, void* context)
{

    // validate that the data makes sense
    int n_packets;
    vio_data_t* data_array = modal_vio_validate_pipe_data(data, bytes, &n_packets);
    if(data_array == NULL) return;

    VIOInterface *interface = (VIOInterface *) context;
    ros::Publisher posePublisher  = interface->GetPosePublisher();
    ros::Publisher odomPublisher  = interface->GetOdometryPublisher();
    ros::Publisher statePublisher = interface->GetStatePublisher();

    geometry_msgs::PoseStamped*       poseMsg  = interface->GetPoseMsg();
    nav_msgs::Odometry*               odomMsg  = interface->GetOdometryMsg();
    voxl_mpa_to_ros::InternalStates*  StateMsg = interface->GetStateMsg();

    // make a new data struct to hold the average
    imu_data_t avg;
    memset(&avg,0,sizeof(avg));

    // sum all the samples
    for(int i=0;i<n_packets;i++){geometry_msgs::TransformStamped grav_to_imu_start;
      tf2::Vector3 grav(vio_pose.gravity[0],
                        vio_pose.gravity[1],
                        vio_pose.gravity[2]);
      tf2::Vector3 unit_z(0,0,1);
      tf2::Quaternion q_grav( grav.cross(unit_z), grav.angle(unit_z));
      if(snav_mode_)
      {
        tf2::convert(q_grav.inverse(),grav_to_imu_start.transform.rotation);
        grav_to_imu_start.child_frame_id = "grav";
        grav_to_imu_start.header.frame_id = "imu_start";
      }
      else
      {
        tf2::convert(q_grav,grav_to_imu_start.transform.rotation);
        grav_to_imu_start.child_frame_id = "imu_start";
        grav_to_imu_start.header.frame_id = "grav";
      }
      grav_to_imu_start.header.stamp = vio_timestamp;
      transforms.push_back(grav_to_imu_start);

      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = "imu_start";
      pose_msg.header.stamp = vio_timestamp;
      pose_msg.header.seq = vio_frame_id;
      // translate VIO pose to ROS pose
      tf2::Matrix3x3 R(
        vio_pose.bodyPose.matrix[0][0],
        vio_pose.bodyPose.matrix[0][1],
        vio_pose.bodyPose.matrix[0][2],
        vio_pose.bodyPose.matrix[1][0],
        vio_pose.bodyPose.matrix[1][1],
        vio_pose.bodyPose.matrix[1][2],
        vio_pose.bodyPose.matrix[2][0],
        vio_pose.bodyPose.matrix[2][1],
        vio_pose.bodyPose.matrix[2][2]);
      tf2::Quaternion q;
      R.getRotation(q);
      pose_msg.pose.position.x = vio_pose.bodyPose.matrix[0][3];
      pose_msg.pose.position.y = vio_pose.bodyPose.matrix[1][3];
      pose_msg.pose.position.z = vio_pose.bodyPose.matrix[2][3];
      pose_msg.pose.orientation.x = q.getX();
      pose_msg.pose.orientation.y = q.getY();
      pose_msg.pose.orientation.z = q.getZ();
      pose_msg.pose.orientation.w = q.getW();
      vio_pose_publisher_.publish(pose_msg);

      geometry_msgs::TransformStamped imu_start_to_imu;
      imu_start_to_imu.transform.translation.x = pose_msg.pose.position.x;
      imu_start_to_imu.transform.translation.y = pose_msg.pose.position.y;
      imu_start_to_imu.transform.translation.z = pose_msg.pose.position.z;
      imu_start_to_imu.transform.rotation.x = pose_msg.pose.orientation.x;
      imu_start_to_imu.transform.rotation.y = pose_msg.pose.orientation.y;
      imu_start_to_imu.transform.rotation.z = pose_msg.pose.orientation.z;
      imu_start_to_imu.transform.rotation.w = pose_msg.pose.orientation.w;
      imu_start_to_imu.child_frame_id = "imu";
      imu_start_to_imu.header.frame_id = "imu_start";
      imu_start_to_imu.header.stamp = vio_timestamp;

      if(snav_mode_)
      {
        tf2::Transform vio_tf, vio_tf_inv;
        tf2::convert(imu_start_to_imu.transform, vio_tf);
        vio_tf_inv = vio_tf.inverse();
        geometry_msgs::TransformStamped imu_to_imu_start;
        tf2::convert(vio_tf_inv, imu_to_imu_start.transform);
        imu_to_imu_start.child_frame_id = "imu_start";
        imu_to_imu_start.header.frame_id = "imu";
        imu_to_imu_start.header.stamp = vio_timestamp;
        transforms.push_back(imu_to_imu_start);
      }
      else
      {
        transforms.push_back(imu_start_to_imu);
      }
      
      tf_broadcaster_.sendTransform(transforms);

      nav_msgs::Odometry odom_msg;
      odom_msg.header.stamp = vio_timestamp;
      odom_msg.header.frame_id = "imu_start";
      odom_msg.child_frame_id = "imu";
      odom_msg.pose.pose = pose_msg.pose;
      odom_msg.twist.twist.linear.x = vio_pose.velocity[0];
      odom_msg.twist.twist.linear.y = vio_pose.velocity[1];
      odom_msg.twist.twist.linear.z = vio_pose.velocity[2];
      odom_msg.twist.twist.angular.x = vio_pose.angularVelocity[0];
      odom_msg.twist.twist.angular.y = vio_pose.angularVelocity[1];
      odom_msg.twist.twist.angular.z = vio_pose.angularVelocity[2];
      //set the error covariance for the pose.
      //initialize twist covariance to zeros.
      for( int16_t i = 0; i < 6; i++ ) {
        for( int16_t j = 0; j < 6; j++ ) {
          odom_msg.pose.covariance[ i*6 + j ] = vio_pose.errCovPose[i][j];
          odom_msg.twist.covariance[ i*6 + j ] = 0.0;
        }
      }
      //set the error covariance for the velocity.
      for( int16_t i = 0; i < 3; i++ ) {
        for( int16_t j = 0; j < 3; j++ ) {
          odom_msg.twist.covariance[ i*6 + j ] = vio_pose.errCovVelocity[i][j];
        }
      }
      vio_odom_publisher_.publish(odom_msg);

      // Internal States
      snap_msgs::InternalStates states_msg;
      states_msg.header.frame_id = "imu_start";
      states_msg.header.stamp = vio_timestamp;
      states_msg.header.seq = vio_frame_id;
      states_msg.snav_mode = snav_mode_;
      states_msg.pose_quality = vio_pose.poseQuality;

      // gravity_camera_pose
      tf2::Matrix3x3 Rgcp(
        vio_pose.gravityCameraPose.matrix[0][0],
        vio_pose.gravityCameraPose.matrix[0][1],
        vio_pose.gravityCameraPose.matrix[0][2],
        vio_pose.gravityCameraPose.matrix[1][0],
        vio_pose.gravityCameraPose.matrix[1][1],
        vio_pose.gravityCameraPose.matrix[1][2],
        vio_pose.gravityCameraPose.matrix[2][0],
        vio_pose.gravityCameraPose.matrix[2][1],
        vio_pose.gravityCameraPose.matrix[2][2]);
      tf2::Quaternion qgcp;
      Rgcp.getRotation(qgcp);
      states_msg.gravity_camera_pose.translation.x = vio_pose.gravityCameraPose.matrix[0][3];
      states_msg.gravity_camera_pose.translation.y = vio_pose.gravityCameraPose.matrix[1][3];
      states_msg.gravity_camera_pose.translation.z = vio_pose.gravityCameraPose.matrix[2][3];
      states_msg.gravity_camera_pose.rotation.x = qgcp.getX();
      states_msg.gravity_camera_pose.rotation.y = qgcp.getY();
      states_msg.gravity_camera_pose.rotation.z = qgcp.getZ();
      states_msg.gravity_camera_pose.rotation.w = qgcp.getW();

      // time_alignment
      states_msg.time_alignment.data = ros::Duration(vio_pose.timeAlignment);

      // gravity
      states_msg.gravity.x = vio_pose.gravity[0];
      states_msg.gravity.y = vio_pose.gravity[1];
      states_msg.gravity.z = vio_pose.gravity[2];

      // err_cov_gravity
      std::copy(&vio_pose.errCovGravity[0][0], &vio_pose.errCovGravity[0][0]+9,
            states_msg.err_cov_gravity.begin());

      // gyro_bias
      states_msg.gyro_bias.x = vio_pose.wBias[0];
      states_msg.gyro_bias.y = vio_pose.wBias[1];
      states_msg.gyro_bias.z = vio_pose.wBias[2];

      // accel_bias
      states_msg.accel_bias.x = vio_pose.aBias[0];
      states_msg.accel_bias.y = vio_pose.aBias[1];
      states_msg.accel_bias.z = vio_pose.aBias[2];

      // R_gyro_body
      tf2::Matrix3x3 Rgb(
        vio_pose.Rbg[0][0], vio_pose.Rbg[0][1], vio_pose.Rbg[0][2],
        vio_pose.Rbg[1][0], vio_pose.Rbg[1][1], vio_pose.Rbg[1][2],
        vio_pose.Rbg[2][0], vio_pose.Rbg[2][1], vio_pose.Rbg[2][2]);
      tf2::Quaternion qgb;
      Rgb.getRotation(qgb);
      states_msg.R_gyro_body.x = qgb.getX();
      states_msg.R_gyro_body.y = qgb.getY();
      states_msg.R_gyro_body.z = qgb.getZ();
      states_msg.R_gyro_body.w = qgb.getW();

      // a_accel_inv, a_gyro_inv
      std::copy(&vio_pose.aAccInv[0][0], &vio_pose.aAccInv[0][0]+9, states_msg.a_accel_inv.begin());
      std::copy(&vio_pose.aGyrInv[0][0], &vio_pose.aGyrInv[0][0]+9, states_msg.a_gyro_inv.begin());

      // tf_imu_camera
      tf2::Matrix3x3 Rbc(
        vio_pose.Rbc[0][0], vio_pose.Rbc[0][1], vio_pose.Rbc[0][2],
        vio_pose.Rbc[1][0], vio_pose.Rbc[1][1], vio_pose.Rbc[1][2],
        vio_pose.Rbc[2][0], vio_pose.Rbc[2][1], vio_pose.Rbc[2][2]);
      tf2::Quaternion qbc;
      Rbc.getRotation(qbc);
      states_msg.tf_imu_camera.translation.x = vio_pose.tbc[0];
      states_msg.tf_imu_camera.translation.y = vio_pose.tbc[1];
      states_msg.tf_imu_camera.translation.z = vio_pose.tbc[2];
      states_msg.tf_imu_camera.rotation.x = qbc.getX();
      states_msg.tf_imu_camera.rotation.y = qbc.getY();
      states_msg.tf_imu_camera.rotation.z = qbc.getZ();
      states_msg.tf_imu_camera.rotation.w = qbc.getW();

      // error_code
      states_msg.error_code = vio_pose.errorCode;

      vio_states_publisher_.publish(states_msg);

    }


    return;
}
