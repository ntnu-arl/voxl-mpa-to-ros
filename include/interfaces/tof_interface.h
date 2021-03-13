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
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>

#include "generic_interface.h"

#define NUM_TOF_REQUIRED_CHANNELS 4

#define TOF_PC_CHANNEL    (m_baseChannel)
#define TOF_IR_CHANNEL    (m_baseChannel + 1)
#define TOF_NOISE_CHANNEL (m_baseChannel + 2)
#define TOF_CONF_CHANNEL  (m_baseChannel + 3)

#define FLIPIMAGE true

#define TofImageWidth  224
#define TofImageHeight 172
#define TofImageSize   (TofImageHeight * TofImageWidth)

typedef struct PointCloud{
    float points[TofImageSize * 3];
    float noise[TofImageSize];
    uint8_t conf[TofImageSize];
    uint8_t ir[TofImageSize];
}PointCloud;

typedef struct RingNode {
    PointCloud pc;
    struct RingNode *next;
    int flag = 0;
    int64_t timestamp_ns = -1;
} RingNode;

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

    void InitializeCameraInfoMessage();

    sensor_msgs::CameraInfo GetCamInfo(){
        return m_cameraInfoMsg;
    }

    sensor_msgs::Image* GetIRMsg(){
        return m_irImageMsg;
    }

    image_transport::CameraPublisher GetIRPublisher(){
        return m_irImagePublisher;
    }

private:

    sensor_msgs::CameraInfo                m_cameraInfoMsg;                   ///< Camera Info message

    sensor_msgs::Image*                    m_irImageMsg;                 ///< Image message
    image_transport::CameraPublisher       m_irImagePublisher;          ///< Image publisher

    sensor_msgs::Image*                    m_confImageMsg;                 ///< Image message
    image_transport::CameraPublisher       m_confImagePublisher;          ///< Image publisher

    sensor_msgs::Image*                    m_noiseImageMsg;                 ///< Image message
    image_transport::CameraPublisher       m_noiseImagePublisher;          ///< Image publisher

    char                                   m_baseName [MODAL_PIPE_MAX_PATH_LEN];
    char                                   m_irName   [MODAL_PIPE_MAX_PATH_LEN];
    char                                   m_confName [MODAL_PIPE_MAX_PATH_LEN];
    char                                   m_noiseName[MODAL_PIPE_MAX_PATH_LEN];

};
#endif
