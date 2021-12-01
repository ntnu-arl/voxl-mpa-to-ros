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

#ifndef OBJECT_DETECT_MPA_INTERFACE
#define OBJECT_DETECT_MPA_INTERFACE

#include <ros/ros.h>
#include <voxl_mpa_to_ros/ObjectDetection.h>
#include "generic_interface.h"


typedef struct object_detection_msg {
    int64_t timestamp_ns;
    uint32_t class_id;
    char class_name[64];
    float class_confidence;
    float detection_confidence;
    float x_min;
    float y_min;
    float x_max;
    float y_max;
} __attribute__((packed)) object_detection_msg;

typedef struct detections_array {
    int32_t num_detections;
    object_detection_msg detections[64];
} __attribute__((packed)) detections_array;


class ObjectDetectInterface: public GenericInterface
{
public:
    ObjectDetectInterface(ros::NodeHandle  rosNodeHandle,
                          ros::NodeHandle  rosNodeHandleParams,
                          const char *     pipeName);

    ~ObjectDetectInterface() { };


    int  GetNumClients();
    void AdvertiseTopics();
    void StopAdvertising();

    voxl_mpa_to_ros::ObjectDetection& GetObjMsg(){
        return m_objMsg;
    }

    ros::Publisher& GetPublisher(){
        return m_rosPublisher;
    }

private:

    voxl_mpa_to_ros::ObjectDetection m_objMsg;
    ros::Publisher                 m_rosPublisher;          ///< Obj publisher

};

#endif //OBJECT_DETECT_MPA_INTERFACE
