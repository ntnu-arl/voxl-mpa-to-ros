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

#include <math.h>
#include <modal_pipe.h>
#include <string> 
#include <sensor_msgs/image_encodings.h>
#include "common_utils.h"

int rotation_to_quaternion(float R[3][3], double* q)
{
    float t,s;
    float trace = R[0][0] + R[1][1] + R[2][2];

    if(trace > 0.0){
        s = sqrt(trace + 1.0);
        q[0] = 0.5 * s;
        s = 0.5 / s;
        q[1] = (R[1][2] - R[2][1]) * s;
        q[2] = (R[2][0] - R[0][2]) * s;
        q[3] = (R[0][1] - R[1][0]) * s;
    }
    // algorithm courtesy of Mike Day
    // https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
    if (R[2][2] < 0){
        if(R[0][0] >R[1][1]){
            t= 1 + R[0][0] - R[1][1] - R[2][2];
            s = (0.5 / sqrt(t));
            q[0] = (R[1][2] - R[2][1]) * s;
            q[1] = t*s;
            q[2] = (R[0][1] + R[1][0]) * s;
            q[3] = (R[2][0] + R[0][2]) * s;
        }else{
            t= 1 - R[0][0] + R[1][1] - R[2][2];
            s = (0.5 / sqrt(t));
            q[0] = (R[2][0] - R[0][2]) * s;
            q[1] = (R[0][1] + R[1][0]) * s;
            q[2] = t*s;
            q[3] = (R[1][2] + R[2][1]) * s;
        }
    }else{
        if(R[0][0] < -R[1][1]){
            t= 1 - R[0][0] - R[1][1] + R[2][2];
            s = (0.5 / sqrt(t));
            q[0] = (R[0][1] - R[1][0]) * s;
            q[1] = (R[2][0] + R[0][2]) * s;
            q[2] = (R[1][2] + R[2][1]) * s;
            q[3] = t*s;
        }else{
            t= 1 + R[0][0] + R[1][1] + R[2][2];
            s = (0.5 / sqrt(t));
            q[0] = t*s;
            q[1] = (R[1][2] - R[2][1]) * s;
            q[2] = (R[2][0] - R[0][2]) * s;
            q[3] = (R[0][1] - R[1][0]) * s;
        }
    }
    return 0;
}

// convert a monotonic clock time in nanoseconds to an equivalent ros::time
// which is really clock_realtime, so an offset is calculated.
ros::Time _clock_monotonic_to_ros_time(uint64_t monotonic_ns)
{
    // get current time in both monotonic and ros
    ros::Time ros_time_now = ros::Time::now();
    ros::Time monotonic_time_now = ros::Time().fromNSec(_time_monotonic_ns());
    ros::Duration offset = ros_time_now - monotonic_time_now;
    return ros::Time().fromNSec(monotonic_ns) + offset;
}

int64_t _time_monotonic_ns()
{
    struct timespec ts;
    if(clock_gettime(CLOCK_MONOTONIC, &ts)){
        fprintf(stderr,"ERROR calling clock_gettime\n");
        return -1;
    }
    return (int64_t)ts.tv_sec*1000000000 + (int64_t)ts.tv_nsec;
}


int64_t _time_realtime_ns()
{
    struct timespec ts;
    if(clock_gettime(CLOCK_REALTIME, &ts)){
        fprintf(stderr,"ERROR calling clock_gettime\n");
        return -1;
    }
    return (int64_t)ts.tv_sec*1000000000 + (int64_t)ts.tv_nsec;
}
