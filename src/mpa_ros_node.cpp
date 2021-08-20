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

#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <ros/ros.h>
#include <modal_pipe.h>
#include "all_interfaces.h"
#include "interface_manager.h"

//Any additional potential interfaces should be added here, see 
//potentialInterface struct for reference
#define NUM_POTENTIAL_INTERFACES 20
#define POTENTIAL_INTERFACES {\
    {"image0_pipe",    "image0_publish",    INT_CAMERA}, \
    {"image1_pipe",    "image1_publish",    INT_CAMERA}, \
    {"image2_pipe",    "image2_publish",    INT_CAMERA}, \
    {"image3_pipe",    "image3_publish",    INT_CAMERA}, \
    {"image4_pipe",    "image4_publish",    INT_CAMERA}, \
    {"image5_pipe",    "image5_publish",    INT_CAMERA}, \
    {"image6_pipe",    "image6_publish",    INT_CAMERA}, \
    {"image7_pipe",    "image7_publish",    INT_CAMERA}, \
    {"stereo0_pipe",   "stereo0_publish",   INT_STEREO}, \
    {"stereo1_pipe",   "stereo1_publish",   INT_STEREO}, \
    {"tof0_pipe",      "tof0_publish",      INT_TOF},    \
    {"tof1_pipe",      "tof1_publish",      INT_TOF},    \
    {"imu0_pipe",      "imu0_publish",      INT_IMU},    \
    {"imu1_pipe",      "imu1_publish",      INT_IMU},    \
    {"vio0_pipe",      "vio0_publish",      INT_VIO},    \
    {"vio1_pipe",      "vio1_publish",      INT_VIO},    \
    {"PC0_pipe",       "PC0_publish",       INT_PC},     \
    {"PC1_pipe",       "PC1_publish",       INT_PC},     \
    {"PC2_pipe",       "PC2_publish",       INT_PC},     \
    {"PC3_pipe",       "PC3_publish",       INT_PC}      \
    }



InterfaceManager *manager = NULL;
GenericInterface *interfaces[NUM_POTENTIAL_INTERFACES];
int              numInterfaces = 0;

typedef struct PotentialInterface{

    const std::string pipeArg;     //string rosparam to look for what pipe
    const std::string publishArg;  //boolean rosparam to look for whether to publish
    const InterfaceType type;

}PotentialInterface;

int MainEnter(int argc, char **argv, ros::NodeHandle nh, ros::NodeHandle nhp){

    int channel = 0;
    PotentialInterface potentials[NUM_POTENTIAL_INTERFACES] = POTENTIAL_INTERFACES;

    for(int i = 0; i < NUM_POTENTIAL_INTERFACES; i++){

        PotentialInterface pInt = potentials[i];

        bool pub;
        nhp.param<bool>(pInt.publishArg, pub, false);
        if(pub){

            std::string pipeName;
            nhp.getParam(pInt.pipeArg, pipeName);

            switch (pInt.type){
                case INT_CAMERA:
                    interfaces[numInterfaces] = new CameraInterface(nh, nhp, channel, pipeName.c_str());
                    break;
                case INT_STEREO:
                    interfaces[numInterfaces] = new StereoInterface(nh, nhp, channel, pipeName.c_str());
                    break;
                case INT_TOF:
                    interfaces[numInterfaces] = new TofInterface(nh, nhp, channel, pipeName.c_str());
                    ((TofInterface*)interfaces[numInterfaces])->SetThreshold(100);
                    break;
                case INT_IMU:
                    interfaces[numInterfaces] = new IMUInterface(nh, nhp, channel, pipeName.c_str());
                    break;
                case INT_VIO:
                    interfaces[numInterfaces] = new VIOInterface(nh, nhp, channel, pipeName.c_str());
                    break;
                case INT_PC:
                    interfaces[numInterfaces] = new PointCloudInterface(nh, nhp, channel, pipeName.c_str());
                    break;
                default:
                    printf("Invalid interface type specified for pipe: %s, exiting\n", pipeName.c_str());
                    return -1;
            }

            channel += interfaces[numInterfaces]->GetNumRequiredChannels();
            numInterfaces++;

        } else {
            printf("Param: \"%s\" set to false, not publishing associated interface\n", pInt.publishArg.c_str());
        }

    }

    if(numInterfaces == 0){
        printf("No combinations of requested pipes and existing pipes found, rosnode exiting\n");
        return -1;
    }

    manager = new InterfaceManager(interfaces, numInterfaces);

    manager->Start();

    return 0;

}

void MainExit(){

    if(manager != NULL){
        manager->Stop();

        delete manager;
    }

    for(int i = 0; i < numInterfaces; i++)
        delete interfaces[i];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voxl_mpa_to_ros_node");
    ros::NodeHandle nhtopics("");
    ros::NodeHandle nhparams("~");
    if(MainEnter(argc, argv, nhtopics, nhparams)){
        MainExit();
        return -1;
    }

    printf("\n\nMPA to ROS app is now running\n\n");
    fflush(stdout);

    ros::spin();
    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();
    ros::waitForShutdown();

    printf("\nMPA to ROS app is now stopping\n\n");
    MainExit();
    printf("\nMPA to ROS app is done\n\n");

    return 0;
}
