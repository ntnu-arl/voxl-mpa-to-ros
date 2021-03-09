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

#ifndef GENERIC_MPA_INTERFACE
#define GENERIC_MPA_INTERFACE

#include <string.h>
#include <ros/ros.h>
#include <modal_pipe.h>

#define PIPE_CLIENT_NAME "mpa-to-ros"

enum InterfaceState {
    ST_NULL,
    ST_READY,
    ST_AD,
    ST_RUNNING,
    ST_CLEAN
};

/**
 * This class describes the functionality for a generic mpa interface, all interfaces
 * should follow this structure, see camera_interface for a basic example
 * 
 * All children should call the 4-parameter constructor of this class
 * 
 * rosNodeHandle and basechannel should be supplied by main, numchannels should 
 * be supplied by the child class, and pipename should be supplied by main via a rosparam
 */
class GenericInterface
{

public:

    GenericInterface(ros::NodeHandle  rosNodeHandle,
                     int              baseChannel,
                     const int        numChannels,    //Numer of MPA channels this interface requires
                     const char *     pipeName):
    m_rosNodeHandle(rosNodeHandle),
    m_baseChannel(baseChannel),
    m_numRequiredChannels(numChannels)
    {
        strcpy(m_pipeName, pipeName);
        m_state = ST_READY;
    }

    virtual ~GenericInterface(){};


    /**
     * The interface should inspect all of its output messages and determine
     * if any have subscribers
     *
     * @return     True if this interface has ros subscribers, false otherwise
     */
    virtual int GetNumClients()        = 0;

    /**
     * The interface should use this function to inspect which of 
     * its potential pipes are available and advertise them to ros
     * 
     * state should be set to advertising at the end of a successful call
     */
    virtual void AdvertiseTopics()     = 0;

    /**
     * The interface should use this function to subscribe to its pipes 
     * and begin actually publishing data to ros
     * 
     * This may be called multiple times during execution
     * 
     * state should be set to running at the end of a successful call
     */
    virtual void StartPublishing()     = 0;

    /**
     * The interface should use this function to unsubscribe from its
     * pipes and cease publishing data to ros
     * 
     * This may be called multiple times during execution
     * 
     * state should be set to advertising at the end of a successful call
     */
    virtual void StopPublishing()      = 0;

    /**
     * The interface should use this function to close all ros outputs and
     * mpa pipes
     * 
     * state should be set to clean at the end of a successful call
     */
    virtual void CleanAndExit()        = 0;

    InterfaceState GetState(){
        return m_state;
    }

    const int GetNumRequiredChannels(){
        return m_numRequiredChannels;
    }

    char *GetPipeName(){
        return (char *)m_pipeName;
    }

protected:

    const ros::NodeHandle   m_rosNodeHandle;
    const int               m_baseChannel         = 0;
    const int               m_numRequiredChannels = 0;
    char                    m_pipeName[MODAL_PIPE_MAX_PATH_LEN];
    InterfaceState          m_state               = ST_NULL;

};

#endif