/*******************************************************************************
 * Copyright 2020 ModalAI Inc.
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

#ifndef TOF_MPA_INTERFACE
#define TOF_MPA_INTERFACE

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>

#include "generic_interface.h"

#define NUM_TOF_REQUIRED_CHANNELS 1

#define FLIPIMAGE true

#define TofImageWidth  224
#define TofImageHeight 172
#define TofImageSize   (TofImageHeight * TofImageWidth)

class TofInterface: public GenericInterface
{
public:
    TofInterface(ros::NodeHandle rosNodeHandle,
                 int             baseChannel,
                 const char*     camName);

    ~TofInterface() { };

    int  GetNumClients();
    void AdvertiseTopics();
    void StartPublishing();
    void StopPublishing();
    void Clean();

    void InitializeCameraInfoMessage(const char *frame_id);
    void InitializePointCloudMessage(const char *frame_id);

    sensor_msgs::CameraInfo GetCamInfo(){
        return m_cameraInfoMsg;
    }

    sensor_msgs::Image GetIRMsg(){
        return m_irImageMsg;
    }

    image_transport::CameraPublisher GetIRPublisher(){
        return m_irImagePublisher;
    }

    sensor_msgs::Image GetDepthMsg(){
        return m_depthImageMsg;
    }

    image_transport::CameraPublisher GetDepthPublisher(){
        return m_depthImagePublisher;
    }

    sensor_msgs::Image GetConfMsg(){
        return m_confImageMsg;
    }

    image_transport::CameraPublisher GetConfPublisher(){
        return m_confImagePublisher;
    }

    sensor_msgs::Image GetNoiseMsg(){
        return m_noiseImageMsg;
    }

    image_transport::CameraPublisher GetNoisePublisher(){
        return m_noiseImagePublisher;
    }

    sensor_msgs::PointCloud2 GetPCMsg(){
        return m_pcMsg;
    }
    ros::Publisher GetPCPublisher(){
        return m_pcPublisher;
    }

    int                                    m_pcConfThreshold;              ///< Confidence thrshold to not publish points

private:
    sensor_msgs::CameraInfo                m_cameraInfoMsg;                ///< Camera Info message

    sensor_msgs::Image                     m_irImageMsg;                   ///< IR Image message
    image_transport::CameraPublisher       m_irImagePublisher;             ///< IR Image publisher

    sensor_msgs::Image                     m_depthImageMsg;                ///< Depth Image message
    image_transport::CameraPublisher       m_depthImagePublisher;          ///< Depth Image publisher

    sensor_msgs::Image                     m_confImageMsg;                 ///< Confidence Image message
    image_transport::CameraPublisher       m_confImagePublisher;           ///< Confidence Image publisher

    sensor_msgs::Image                     m_noiseImageMsg;                ///< Noise Image message
    image_transport::CameraPublisher       m_noiseImagePublisher;          ///< Noise Image publisher

    sensor_msgs::PointCloud2               m_pcMsg;                        ///< Point cloud message
    ros::Publisher                         m_pcPublisher;                  ///< Point cloud publisher



};
#endif
