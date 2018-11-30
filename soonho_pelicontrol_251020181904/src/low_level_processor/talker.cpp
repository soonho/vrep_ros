#include "ros/ros.h"
#include "CtrlInput.h"
#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitions

#include "asctec.h"

int initComms()
{
	char buf[512];
	int i;
	POLL_HEADER* pHead;
	POLL_FOOTER* pFoot;
	struct termios port_settings; // structure to store the port settings in
	int fd;
	int size;

	// Initializing the Serial Port
	fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if (fd == -1)
	{
		printf("ERROR: Can't open /dev/ttyUSB0. \n");
		return -1;
	}
	else
	{
		printf("/dev/ttyUSB1 opened correctly.\n");
	};

	cfsetispeed(&port_settings, B57600);    // set baud rates
	port_settings.c_cflag = B57600 | CS8 | CREAD | CLOCAL;
	port_settings.c_iflag = IGNPAR;
	port_settings.c_oflag = 0;
	port_settings.c_lflag = 0;
	tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port

	// This requests LL Status and IMU CalcData
	// 0x0001 + 0x0004 = 0x0005
	POLL_REQUEST req = { {'>', '*', '>', 'p'}, 0x0005 };

	size = write(fd, &req, sizeof(POLL_REQUEST));
	fsync(fd);
	printf("%d bytes written.\n", size);
	if (size == 0)
	{
		printf("ERROR: 0 bytes written. Return. \n");
		return -1;
	}

	// wait until data arrives
	usleep(250000);
	
	// now read the data
	size = read(fd, buf, sizeof(buf));

	if (size == -1)
	{
		printf("ERROR: Reading error. Return.\n");
		return -1;
	}
	else if (size == 0)
	{
		printf("ERROR: Nothing to read. Return. \n");
		return -1;
	}

	// This loops through all received packets and prints their infos
	for (i = 0; i < size;)
	{
		pHead = (POLL_HEADER*) &buf[i];
		printf("Start: %.3s\n", pHead->startstring);
		printf("packet desc: %hhu / 0x%02hhX\n", pHead->packet_desc, pHead->packet_desc);
		printf("length: %hu\n", pHead->length);

		i += sizeof(POLL_HEADER);

		switch (pHead->packet_desc)
		{
			case 0x03:	// PD_IMUCALCDATA
			{
				IMU_CALCDATA* pCalcData = (IMU_CALCDATA*)&buf[i];
				printf("angle_nick: %d\n", pCalcData->angle_nick);
				printf("angle_roll: %d\n", pCalcData->angle_roll);
				printf("angle_yaw: %d\n", pCalcData->angle_yaw);
			}
			break;
		}

		i += pHead->length;

		pFoot = (POLL_FOOTER*) &buf[i];
		printf("crc: %04hX\n", pFoot->crc16);
		printf("stop string: %.3s\n\n", pFoot->stopstring);

		i += sizeof(POLL_FOOTER);
	}

	close(fd);
	return 0;
}

int main(int argc, char **argv)
{
	if (initComms() != 0) {
		ROS_INFO("Erro de comunicação!");
		return  -1;
	}
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;

	n.setParam("/soonho/thrust", 0.0);
	n.setParam("/soonho/pitch", 0.0);
	n.setParam("/soonho/roll", 0.0);
	n.setParam("/soonho/yaw", 0.0);

	
	ros::Publisher thrust_pub = n.advertise<asctec_msgs::CtrlInput>("/asctec/CTRL_INPUT", 1);

	ros::Rate loop_rate(30);

//		asctec_msgs::CtrlInput msg;
//		msg.pitch = 0;
//		msg.roll = 0;
//		msg.thrust = 0;
//		msg.yaw = -2047;
//		msg.ctrl = 0xF;
//		msg.chksum = msg.pitch + msg.roll + msg.yaw + msg.thrust + msg.ctrl + (short)0xAAAA;
//		ROS_INFO("chksum", msg.chksum);
//		thrust_pub.publish(msg);

	while (ros::ok())
	{
		asctec_msgs::CtrlInput msg;
		double s_thrust, s_pitch, s_yaw, s_roll;
		if (n.getParam("/soonho/pitch", s_pitch)) {
			msg.pitch = s_pitch;
		} else {
			msg.pitch = 0;
		}
		if (n.getParam("/soonho/roll", s_roll)) {
			msg.roll = s_roll;
		} else {
			msg.roll = 0;
		}
		if (n.getParam("/soonho/thrust", s_thrust)) {
			msg.thrust = s_thrust;
		} else {
			msg.thrust = 0;
		}
		if (n.getParam("/soonho/yaw", s_yaw)) {
			msg.yaw = s_yaw;
		} else {
			msg.yaw = 0;
		}
		msg.ctrl = 0xF;
		msg.chksum = msg.pitch + msg.roll + msg.yaw + msg.thrust + msg.ctrl + (short)0xAAAA;
//		ROS_INFO("chksum", msg.chksum);
		thrust_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}