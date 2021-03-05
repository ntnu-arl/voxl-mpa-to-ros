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

#include <stdio.h>
#include "generic_interface.h"
#include "interface_manager.h"

#define THREAD_INTERVAL_MS 1000

static void* ThreadManageInterfaces(void *pData);

// -----------------------------------------------------------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------------------------------------------------------
InterfaceManager::InterfaceManager(GenericInterface** interfaces,
                                   int                numInterfaces)
{
    
    m_threadData.manager = this;
    m_threadData.running = false;

    m_interfaces         = interfaces;
    m_numInterfaces      = numInterfaces;

}

// -----------------------------------------------------------------------------------------------------------------------------
// This function opens the camera and starts sending the capture requests
// -----------------------------------------------------------------------------------------------------------------------------
void InterfaceManager::Start()
{

    m_threadData.running = true;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    pthread_create(&m_thread, &attr, ThreadManageInterfaces, &m_threadData);
    pthread_attr_destroy(&attr);

}

// -----------------------------------------------------------------------------------------------------------------------------
// This function informs the per camera manager to stop the camera and stop sending any more requests
// -----------------------------------------------------------------------------------------------------------------------------
void InterfaceManager::Stop()
{
    m_threadData.running = false;

    pthread_join(m_thread, NULL);
}

static void* ThreadManageInterfaces(void *pData){

    ThreadData*        pThreadData = (ThreadData*)pData;
    InterfaceManager*  manager     = pThreadData->manager;

    printf("Starting Manager Thread with %d interfaces\n",manager->GetNumInterfaces());

    for(int i = 0; i < manager->GetNumInterfaces(); i++){

        GenericInterface *interface = manager->GetInterface(i);

        if(interface->GetState() == ST_READY){
            printf("Interface %d now advertising\n", i);
            interface->AdvertiseTopics();
        }

    }

    while(pThreadData->running){

        for(int i = 0; i < manager->GetNumInterfaces(); i++){

            GenericInterface *interface = manager->GetInterface(i);

            if(interface->GetState() == ST_RUNNING && interface->GetNumClients() == 0){
                printf("Interface %d ceasing to publish\n", i);
                interface->StopPublishing();
                continue;
            }

            if(interface->GetState() == ST_AD && interface->GetNumClients() > 0){
                printf("Interface %d now publishing\n", i);
                interface->StartPublishing();
                continue;
            }

        }

        usleep(THREAD_INTERVAL_MS * 1000);
    }

    for(int i = 0; i < manager->GetNumInterfaces(); i++){

        GenericInterface *interface = manager->GetInterface(i);

        if(interface->GetState() == ST_RUNNING){
            interface->StopPublishing();
        }

        if(interface->GetState() == ST_AD){
            interface->CleanAndExit();
        }

    }

    return NULL;

}
