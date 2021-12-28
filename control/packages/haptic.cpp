/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at:
    http://dsc.sensable.com

Module Name:

  HelloHapticDevice.c

Description:

  This application creates a gravity well, which will attract
  the device towards its center when the device enters its proximity.

*******************************************************************************/
#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#undef UNICODE

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>

#include <string.h>

//#pragma comment(lib,"ws2_32.lib") //Winsock Library

#define BUFLEN 512
#define PORT 8888	//The port on which to listen for incoming data


#include <assert.h>


#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>


#include <stdlib.h>
#include <string.h>

#pragma comment(lib,"ws2_32.lib") //Winsock Library


void mainLoop(void);
int createSocket(int port);

HDCallbackCode HDCALLBACK PositionCallback(void* data);

char buf[BUFLEN];
char startByte[3] = "S";
char endByte[3] = "E";
char charOne = 1;
char bufrec[1];
//har server_addr = ;

/*******************************************************************************
 Main function.
 Initializes the device, starts the schedule, creates a schedule callback
 to handle gravity well forces, waits for the user to press a button, exits
 the application.
*******************************************************************************/

int createSocket(int port)
{
    int sock, err;
    struct sockaddr_in server;

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("ERROR: Socket creation failed\n");
        exit(1);
    }
    printf("Socket created.\n");
    memset((char*)&server, '\0', sizeof(server));
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(port);
    if (bind(sock, (struct sockaddr*)&server, sizeof(server)) < 0)
    {
        printf("ERROR: Bind failed\n");
        exit(1);
    }
    printf("Bind done.\n");

    listen(sock, 3);

    return sock;
}

void closeSocket(int sock)
{
    close(sock);
    return;
}

void sendMsg(int sock, void* msg, int msgsize)
{
    if (write(sock, msg, msgsize) < 0)
    {
        printf("Can't send message.\n");
        closeSocket(sock);
        exit(1);
    }
    printf("Message sent (%d bytes).\n", msgsize);
    return;
}

int main(int argc, char* argv[])
{
    WSADATA wsa;
    SOCKET s, new_socket;
    struct sockaddr_in server, client;
    int c;
    char* message;

    printf("\nInitialising Winsock...");
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        printf("Failed. Error Code : %d", WSAGetLastError());
        return 1;
    }

    printf("Initialised.\n");

    //Create a socket
    if ((s = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
    {
        printf("Could not create socket : %d", WSAGetLastError());
    }

    printf("Socket created\n");

    //Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(8888);

    //Bind
    if (bind(s, (struct sockaddr*)&server, sizeof(server)) == SOCKET_ERROR)
    {
        printf("Bind failed with error code : %d", WSAGetLastError());
    }

    puts("Bind done");

    //Listen to incoming connections
    listen(s, 3);

    //Accept and incoming connection
    puts("Waiting for incoming connections...");

    c = sizeof(struct sockaddr_in);
    new_socket = accept(s, (struct sockaddr*)&client, &c);
    if (new_socket == INVALID_SOCKET)
    {
        printf("accept failed with error code : %d", WSAGetLastError());
    }

    puts("Connection accepted");


    HDErrorInfo error;
    HDSchedulerHandle hPosition;

    /* Initialize the device, must be done before attempting to call any hd
       functions. Passing in HD_DEFAULT_DEVICE causes the default device to be
       initialized. */
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }

    printf("Hello Haptic Device!\n");
    printf("Found device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));

    /* Schedule the main callback that will render forces to the device. */
    hPosition = hdScheduleAsynchronous(
        PositionCallback, 0,
        HD_MAX_SCHEDULER_PRIORITY);

    //hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();

    /* Check for errors and abort if so. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        return -1;
    }

    /* Wait until the user presses a key.  Meanwhile, the scheduler
       runs and applies forces to the device. */
    printf("Press any key to quit.\n\n");


    while (!_kbhit())
    {
        //sprintf(buf, "%3.5f;%3.5f;%3.5f;", 0, 1, 2);
        //memset(bufrec, 0, sizeof(bufrec));
        char bufrec[1];
        int rec = recv(new_socket, bufrec, 1, 0);
        //send(s, buf, strlen(buf), 0);
        if (rec > 0) {
            send(new_socket, buf, strlen(buf), 0);
            //printf("Buffer recv: %.*s\n", rec, bufrec);
        }
        //else {
            //puts("Hello World");
            //send(new_socket, buf, strlen(buf), 0);
            //printf("Buffer not received: %s\n", buf);
        //}
    }


    /* For cleanup, unschedule callback and stop the scheduler. */
    hdStopScheduler();
    hdUnschedule(hPosition);

    /* Disable the device. */
    hdDisableDevice(hHD);

    return 0;
}

/*******************************************************************************
 Servo callback.
 Called every servo loop tick.  Simulates a gravity well, which sucks the device
 towards its center whenever the device is within a certain range.
*******************************************************************************/
HDCallbackCode HDCALLBACK PositionCallback(void* data)
{
    //const HDdouble kStiffness = 0.075; /* N/mm */
    //const HDdouble kGravityWellInfluence = 40; /* mm */

    /* This is the position of the gravity well in cartesian
       (i.e. x,y,z) space. */
       //static const hduVector3Dd wellPos = {0,0,0};

    HDErrorInfo error;
    hduVector3Dd position;
    //hduVector3Dd force;
    //hduVector3Dd positionTwell;

    HHD hHD = hdGetCurrentDevice();

    /* Begin haptics frame.  ( In general, all state-related haptics calls
       should be made within a frame. ) */
    hdBeginFrame(hHD);

    /* Get the current position of the device. */
    hdGetDoublev(HD_CURRENT_POSITION, position);

    /* End haptics frame. */
    hdEndFrame(hHD);
    //printf("Position: %f, %f, %f \n", position[0], position[1], position[2]);

    //clear the buffer by filling null, it might have previously received data
    memset(buf, 0, sizeof(buf)-1);
    memset(buf + sizeof(buf) - 1, '\0', 1);

    sprintf(buf, "%s;%3.5f;%3.5f;%3.5f;%s;", startByte, position[0], position[1], position[2], endByte);
    //printf("%s\n", buf);
    /* Check for errors and abort the callback if a scheduler error
       is detected. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error,
            "Error detected while rendering gravity well\n");

        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }

    /* Signify that the callback should continue running, i.e. that
       it will be called again the next scheduler tick. */
    return HD_CALLBACK_CONTINUE;
}

/*****************************************************************************/
