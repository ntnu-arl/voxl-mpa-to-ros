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
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <ros/ros.h>
#include "all_interfaces.h"
#include "interface_manager.h"

InterfaceManager *manager = NULL;
GenericInterface *interfaces[3];

void MainEnter(int argc, char **argv, ros::NodeHandle rosNodeHandle){

    int channel = 0;

    interfaces[0] = new CameraInterface(rosNodeHandle,0, "tracking");
    interfaces[1] = new StereoInterface(rosNodeHandle,1, "stereo");
    //interfaces[2] = new CameraInterface(rosNodeHandle,1, "hires_preview");

    manager = new InterfaceManager(interfaces, 2);

    manager->Start();

}

void MainExit(){

    manager->Stop();

    usleep(1000000);

    delete manager;

    delete &interfaces[0];
    delete &interfaces[1];
    //delete &interfaces[2];

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voxl_mpa_to_ros_node");
    ros::NodeHandle rosNodeHandle("~");
    MainEnter(argc, argv, rosNodeHandle);

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
