/*

Copyright (c) 2011, Markus Achtelik, ASL, ETH Zurich, Switzerland
You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <string.h>
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_ctrl_motors.h>
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Int8.h"

void usage()
{
    std::string text("usage: \n\n");
    text = "ctrl_test motors [0 | 1]\n";
    text += "ctrl_test ctrl [acc | vel | pos | vel_b | pos_b] x y z yaw\n";
    text += "position / velocity in [m] / [m/s] and yaw in [deg] / [deg/s] (-180 ... 180)\n";
    std::cout << text << std::endl;
}

void accInputCallback(const geometry_msgs::Quaternion::ConstPtr& msg){
    ros::NodeHandle n;
    n.setParam("/soonho/x", msg->x);
    n.setParam("/soonho/y", msg->y);
    n.setParam("/soonho/z", msg->z);
    n.setParam("/soonho/yaw", msg->w * M_PI / 180.0);
    n.setParam("/soonho/type", asctec_hl_comm::mav_ctrl::acceleration);
}

void velInputCallback(const geometry_msgs::Quaternion::ConstPtr& msg){
    ros::NodeHandle n;
    n.setParam("/soonho/x", msg->x);
    n.setParam("/soonho/y", msg->y);
    n.setParam("/soonho/z", msg->z);
    n.setParam("/soonho/yaw", msg->w * M_PI / 180.0);
    n.setParam("/soonho/type", asctec_hl_comm::mav_ctrl::velocity);
}

void motorInputCallback(const std_msgs::Int8::ConstPtr& msg){
    asctec_hl_comm::mav_ctrl_motors::Request req;
    asctec_hl_comm::mav_ctrl_motors::Response res;
    req.startMotors = msg->data;
    ros::service::call("fcu/motor_control", req, res);
    std::cout << "motors running: " << (int)res.motorsRunning << std::endl;
}

int main(int argc, char ** argv)
{

    ros::init(argc, argv, "soonho_test");
    ros::NodeHandle nh;

    //topico para leitura de comandos de aceleracao
    ros::Subscriber sub_acc  = nh.subscribe("hl_acc_input", 10, accInputCallback);
    ROS_INFO("Pronto para receber comandos de aceleracao!");
    
    //topico para leitura de comandos de velocidade
    ros::Subscriber sub_vel  = nh.subscribe("hl_vel_input", 10, velInputCallback);
    ROS_INFO("Pronto para receber comandos de velocidade!");
    
    //topico para leitura de comandos de ligar e desligar os motores
    ros::Subscriber sub_motor = nh.subscribe("hl_motor_input", 10, motorInputCallback);
    ROS_INFO("Pronto para receber comandos para ligar e desligar os motores!");
    

    //topico em que sera publicado o comando de controle
    ros::Publisher pub = nh.advertise<asctec_hl_comm::mav_ctrl> ("fcu/control", 1);

    //frequencia para envio de comandos
    ros::Rate loop_rate(30);

    //inicializacao de parametros que serao alterados pelos callbacks e enviados ao fcu/control
    nh.setParam("/soonho/x", 0);
    nh.setParam("/soonho/y", 0);
    nh.setParam("/soonho/z", 0);
    nh.setParam("/soonho/yaw", 0);
    nh.setParam("/soonho/type", asctec_hl_comm::mav_ctrl::acceleration);

    while (ros::ok()) {
        asctec_hl_comm::mav_ctrl msg;
        double s_thrust, s_pitch, s_yaw, s_roll, s_type;
        if (nh.getParam("/soonho/x", s_pitch)) {
            msg.x = s_pitch;
        } else {
            msg.x = 0;
        }
        if (nh.getParam("/soonho/y", s_roll)) {
            msg.y = s_roll;
        } else {
            msg.y = 0;
        }
        if (nh.getParam("/soonho/z", s_thrust)) {
            msg.z = s_thrust;
        } else {
            msg.z = 0;
        }
        if (nh.getParam("/soonho/yaw", s_yaw)) {
            msg.yaw = s_yaw;
        } else {
            msg.yaw = 0;
        }
        if (nh.getParam("/soonho/type", s_type)) {
            msg.type = s_type;
        } else {
            msg.type = 0;
        }
        msg.v_max_xy = -1; // use max velocity from config
        msg.v_max_z = -1;
        
        pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
