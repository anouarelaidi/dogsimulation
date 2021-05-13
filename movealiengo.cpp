/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <sys/time.h> 
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>



using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::A1), udp(level){
        udp.InitCmdData(cmd);
	initdog();
    }
    void initdog();
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    char action = 'n';
    float val = 0;
};
void Custom::initdog() {
	cmd.forwardSpeed = 0.0f;
        cmd.sideSpeed = 0.0f;
        cmd.rotateSpeed = 0.0f;
        cmd.bodyHeight = 0.0f;

        cmd.mode = 0;
        cmd.roll = 0;
        cmd.pitch = 0;
        cmd.yaw = 0;
}

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}


void Custom::RobotControl() 
{
    if (action!='n') {
	    //udp.GetRecv(state);
	    switch (action) {
		case 'f':
		    cmd.mode = 2;
		    cmd.forwardSpeed = val;
		    break;
		case 't':
		    cmd.mode = 2;
		    cmd.rotateSpeed = val;
		    break;
		case 's':
		    cmd.mode = 2;
		    cmd.sideSpeed = val;
		    break;
		case 'r':
		    cmd.mode = 1;
		    cmd.roll = val;
		    break;
		case 'p':
		    cmd.mode = 1;
		    cmd.pitch = val;
		    break;
		case 'y':
		    cmd.mode = 1;
		    cmd.yaw = val;
		    break;
		case 'h':
		    cmd.mode = 1;
		    cmd.bodyHeight = val;
		    break;
		}
	    action='n';
	    udp.SetSend(cmd);
	}
}

int main(int argc, char* argv[])
{
   	//////////////////////////////////////////
	// init dog
	Custom custom(HIGHLEVEL);
	InitEnvironment();
	LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
	LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
	//LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));
	loop_udpSend.start();
	//loop_udpRecv.start();
	loop_control.start();

   	//////////////////////////////////////////
	// server: create socket
     	char buffer[1000];
	int n;

	int serverSock=socket(AF_INET, SOCK_STREAM, 0);

	sockaddr_in serverAddr;
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(13255);
	serverAddr.sin_addr.s_addr = INADDR_ANY;
	bind(serverSock, (struct sockaddr*)&serverAddr, sizeof(struct sockaddr));
	listen(serverSock,1);
	sockaddr_in clientAddr;
	socklen_t sin_size=sizeof(struct sockaddr_in);
	int clientSock=accept(serverSock,(struct sockaddr*)&clientAddr, &sin_size);

	//////////////////////////////////////////
	// server listening and acting
	while (true) {
		bzero(buffer, 1000);

                //receive a message from a client
                n = read(clientSock, buffer, 500);
                
		if (n>2) {
			custom.val=atof(&(buffer[2]));
			if (custom.val<-1)
				custom.val=-1;
			else if (custom.val>1)
				custom.val=1;
			custom.action=buffer[0];
			std::cout << "Server received:  " << custom.action << "  " << custom.val << std::endl;
			
		}
                strcpy(buffer, "done");
                n = write(clientSock, buffer, strlen(buffer));
	};

    return 0; 
}
