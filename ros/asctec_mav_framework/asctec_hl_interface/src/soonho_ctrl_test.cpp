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
#include <tf/tf.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_ctrl_motors.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>

//dados recebidos
geometry_msgs::Quaternion p3dx_01; //dados do p3dx_01
geometry_msgs::Twist pelican_01; //dados do pelican_01

//mensagem para controle do pelican_01
asctec_hl_comm::mav_ctrl control_msg; //asctec
geometry_msgs::Quaternion control_msg_pelican_01; //vrep

//subscribers
ros::Subscriber sub_motor; //comandos de liga/desliga
ros::Subscriber sub_p3dx_01; //subscriber para receber dados do P3DX

//publishers
ros::Publisher pub_pelican_01; //publisher para os comandos de controle do pelican_01
ros::Publisher pub_odom_pelican_01; //publisher para a odometria do pelican_01
ros::Publisher pub_odom_p3dx_01; //publisher para a odometria do p3dx_01

//
double vl_p3dx_01 = 0;
double va_p3dx_01 = 0;

double ux_p3dx_01 = 0;
double uy_p3dx_01 = 0;

double ux_pelican_01 = 0;
double uy_pelican_01 = 0;
double uz_pelican_01 = 0;

double d = 0.2;

double dt = 0.05;


void initVars(){
    p3dx_01.x = 0;
    p3dx_01.y = 0;
    p3dx_01.z = 0;
    p3dx_01.w = 0;

    pelican_01.linear.x = 0;
    pelican_01.linear.y = 0;
    pelican_01.linear.z = 0;
    pelican_01.angular.x = 0;
    pelican_01.angular.y = 0;
    pelican_01.angular.z = 0;

    control_msg_pelican_01.x = 0;
    control_msg_pelican_01.y = 0;
    control_msg_pelican_01.z = 0;
    control_msg_pelican_01.w = 0;
}

//callback para recebimento de dados do p3dx
void p3dx01Callback(const nav_msgs::Odometry::ConstPtr& msg) {
    //dados de posicionamento do p3dx
    p3dx_01.x = msg->pose.pose.position.x;
    p3dx_01.y = msg->pose.pose.position.y;
    p3dx_01.z = msg->pose.pose.position.z;

    //fazer umas contas doidas pra pegar o angulo de rotacao do p3dx
    double quatx = msg->pose.pose.orientation.x;
    double quaty = msg->pose.pose.orientation.y;
    double quatz = msg->pose.pose.orientation.z;
    double quatw = msg->pose.pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    //angulo de rotacao do p3dx
    p3dx_01.w = yaw;
}

//callback para ligar e desligar o motor
void motorInputCallback(const std_msgs::Int8::ConstPtr& msg){
    asctec_hl_comm::mav_ctrl_motors::Request req;
    asctec_hl_comm::mav_ctrl_motors::Response res;
    req.startMotors = msg->data;
    ros::service::call("fcu/motor_control", req, res);
    ROS_INFO("Motores ligados: %d", (int)res.motorsRunning);
}

int main(int argc, char ** argv)
{
    //inicializar as variaveis que guardam os dados de posicionamento
    initVars();
    ros::init(argc, argv, "soonho_test");
    ros::NodeHandle nh;
    
    //topico para leitura de comandos de ligar e desligar os motores
    sub_motor = nh.subscribe("hl_motor_input", 10, motorInputCallback);
    ROS_INFO("Pronto para receber comandos para ligar e desligar os motores!");

    //topico para receber dados do P3DX_01
    sub_p3dx_01  = nh.subscribe("p3dx_01_odom", 10, p3dx01Callback);
    ROS_INFO("Pronto para receber comandos de aceleracao!");

    //topico para receber dados do Pelican_01
    //ros::Subscriber sub_acc  = nh.subscribe("pelican_01_odom", 10);
    //ROS_INFO("Pronto para receber comandos de aceleracao!");
    
    //topico em que sera publicado o comando de controle
    //pub_pelican_01 = nh.advertise<asctec_hl_comm::mav_ctrl> ("fcu/control", 1); //pelican
    pub_pelican_01 = nh.advertise<geometry_msgs::Quaternion> ("/pelican_01/control", 1); //vrep

    //topico em que serao publicados os dados do pelican_01
    pub_odom_pelican_01 = nh.advertise<nav_msgs::Odometry>("/pelican_01_odom",10);

    //topico em que serao publicados os dados do p3dx01
    //pub_odom_p3dx_01 = nh.advertise<nav_msgs::Odometry>("/p3dx_01_odom",10);

    //frequencia para envio de comandos
    ros::Rate loop_rate(30);

    //inicializacao de parametros que serao alterados pelos callbacks e enviados ao fcu/control
    /*
    nh.setParam("/soonho/x", 0);
    nh.setParam("/soonho/y", 0);
    nh.setParam("/soonho/z", 0);
    nh.setParam("/soonho/yaw", 0);
    nh.setParam("/soonho/type", asctec_hl_comm::mav_ctrl::acceleration);
    */

    while (ros::ok()) {
        //TODO: pegar atributos do pelican

        //publicar a odometria do pelican_01
        //pub_odom_pelican_01.publish(pelican_01);

        //calculo de velocidades para o P3DX
	    //ux_p3dx_01 = 0.1*(- p3dx_01->x + pelican_01->linear.x + 0   - 0);
	    //uy_p3dx_01 = 0.1*(- p3dx_01->y + pelican_01->linear.y + 0   - 0);
	    //velocidades para enviar
	    //vl_p3dx_01 = cos(p3dx_01->w) * ux_p3dx_01 + sin(p3dx1_yaw) * uy_p3dx_01;
	    //va_p3dx_01 = (-sin(p3dx_01->w)/d) * ux_p3dx_01 + (cos(p3dx_01->w)/d) * uy_p3dx_01;

	    ux_pelican_01 = 0.1*(- pelican_01.linear.x + p3dx_01.x + 0   - 0 - 2*pelican_01.angular.x);
	    uy_pelican_01 = 0.1*(- pelican_01.linear.y + p3dx_01.y + 0   - 0 - 2*pelican_01.angular.y);
	    uz_pelican_01 = 0.1*(- pelican_01.linear.z + p3dx_01.z + 0.5 - 0 - 2*pelican_01.angular.z);


	    //vel_lin_p3dx1(vl_p3dx1);
	    //vel_ang_p3dx1(va_p3dx1);

	    //accel_x_quad1(ux_quad1);
	    //accel_y_quad1(uy_quad1);
	    //accel_z_quad1(uz_quad1);

	    //publicar comando no vrep
	    control_msg_pelican_01.x = ux_pelican_01;
	    control_msg_pelican_01.y = uy_pelican_01;
	    control_msg_pelican_01.z = uz_pelican_01;
	    //control_msg_pelican_01.w = 0;

        pub_pelican_01.publish(control_msg_pelican_01);

        //publicar comando no asctec pelican
        //control_msg.x = ux_pelican_01;
	    //control_msg.y = uy_pelican_01;
        //control_msg.z = uz_pelican_01;
        //control_msg.yaw = s_yaw;
        //control_msg.type = asctec_hl_comm::mav_ctrl::acceleration; //usando aceleracao
        //control_msg.v_max_xy = -1; // use max velocity from config
        //control_msg.v_max_z = -1;
        //
        //pub_pelican_01.publish(control_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
return 0;
}
