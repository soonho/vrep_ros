#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_ctrl_motors.h>
#include <asctec_hl_comm/mav_imu.h>

using namespace std;

ros::Publisher cmd_accel_pub_quad1;

ros::Subscriber omniant1;
ros::Subscriber quad1;

asctec_hl_comm::mav_ctrl cmd_accel_msg_quad1;

//posicao do pelican
float quad1_x = 0;
float quad1_y = 0;
float quad1_z  = 0;
float quad1_yaw = 0;

//velocidades do pelican
float quad1_vx = 0;
float quad1_vy = 0;
float quad1_vz = 0;
float quad1_vyaw = 0;

//posicao do omniant1
float omniant1_x   = 0;
float omniant1_y   = 0;
float omniant1_z   = 0;
float omniant1_yaw = 0;

//variaveis de controle
float ux = 0;
float uy = 0;
float uz = 0;
float uyaw = 0;

//variaveis para envio ao pelican
float accx = 0;
float accy = 0;
float accz = 0;

//parametros para posicionamento desejado
float target_x = 0;
float target_y = 0;
float target_z = 0;
float target_yaw = 0;

//parametros para ganhos
float gain_x    = 0.5;
float gain_y    = 0.5;
float gain_z    = 1.0;
float gain_yaw  = 1.0;
float gain_vx   = 0.8;
float gain_vy   = 0.8;
float gain_vz   = 0.5;
float gain_vyaw = 0.5;

//parametros para limitadores e offsets
float max_accx = 0;
float max_accy = 0;
float max_accz = 0;
float min_accz = 0;
float max_uyaw = 0;
float offset_z = 0;

void motorInputCallback(int input){
    asctec_hl_comm::mav_ctrl_motors::Request req;
    asctec_hl_comm::mav_ctrl_motors::Response res;
    req.startMotors = input;
    ros::service::call("fcu/motor_control", req, res);
}

void get_data_omniant1 (const nav_msgs::Odometry::ConstPtr& msg)
{
    omniant1_x   = msg->twist.twist.linear.x;
    omniant1_y   = msg->twist.twist.linear.y;
    omniant1_z   = msg->twist.twist.linear.z;
    omniant1_yaw = msg->twist.twist.angular.z;
}

void get_data_quad1 (const nav_msgs::Odometry::ConstPtr& msg)
{
    quad1_vx   = msg->twist.twist.linear.x;
    quad1_vy   = msg->twist.twist.linear.y;
    quad1_vz   = msg->twist.twist.linear.z;
    quad1_vyaw = msg->twist.twist.angular.z;

    quad1_x   = msg->pose.pose.position.x;
    quad1_y   = msg->pose.pose.position.y;
    quad1_z   = msg->pose.pose.position.z;
    quad1_yaw = msg->pose.pose.orientation.w;
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "consensus");
    ros::NodeHandle nh;

    //topico para envio de comandos no pelican
    cmd_accel_pub_quad1 = nh.advertise<asctec_hl_comm::mav_ctrl> ("fcu/control", 1);

    //topico para leitura dos dados do pelican
    quad1 = nh.subscribe("/qsys_quad1", 10, get_data_quad1);

    //topico para leitura dos dados do omniant1
    omniant1 = nh.subscribe("/qsys_omniant1", 10, get_data_omniant1);

    //inicializando parametros de objetivo
    nh.setParam("/soonho/x", 2);
    nh.setParam("/soonho/y", 0);
    nh.setParam("/soonho/z", 0.6);
    nh.setParam("/soonho/yaw", 0);

    //inicializando parametros de ganho
    nh.setParam("/soonho/gain_x", 2);
    nh.setParam("/soonho/gain_y", 0);
    nh.setParam("/soonho/gain_z", 0.6);
    nh.setParam("/soonho/gain_yaw", 0);

    ros::spinOnce();
    ros::Rate loop_rate(20);

    motorInputCallback(1);

    while(ros::ok()) {
        ros::spinOnce();

        //coleta dos parametros do ROS
        if (!nh.getParam("/soonho/x", target_x)) {
            target_x = 2;
        }
        if (!nh.getParam("/soonho/y", target_y)) {
            target_y = 0;
        }
        if (!nh.getParam("/soonho/z", target_z)) {
            target_z = 0.6;
        }
        if (!nh.getParam("/soonho/yaw", target_yaw)) {
            target_yaw = 0;
        }

        //leis de controle
        ux   = 0.5 * (omniant1_x - quad1_x)   - 0.8 * (quad1_vx) + 0 - 1;
        uy   = 0.5 * (omniant1_y - quad1_y)   - 0.8 * (quad1_vy) + 0 - 0;
        uz   = 1.0 * (omniant1_z - quad1_z)   - 0.5 * (quad1_vz) + 1 - 0 + 0.465;
        uyaw = 1.0 * (target_yaw - quad1_yaw) - 0.5 * (quad1_vyaw);

        accx =  ux * cos(quad1_yaw) + uy * sin(quad1_yaw);
        accy = -ux * sin(quad1_yaw) + uy * cos(quad1_yaw);
        accz =  uz;


        if (accx > 0.15) {
            accx = 0.15;
        } else if (accx < -0.15) {
            accx = -0.15;
        }

        if (accy > 0.15) {
            accy = 0.15;
        } else if (accy < -0.15) {
            accy = -0.15;
        }

        if (accz > 0.5) {
            accz = 0.5;
        } else if (accz < 0.465) {
            accz = 0.465;
        }

        if(uyaw > 0) {
            uyaw = uyaw + 0.5;
        }
        else if(uyaw < 0) {
            uyaw = uyaw - 0.5;
        }

        if (uyaw > 0.8) {
            uyaw = 0.8;
        } else if (uyaw < -0.8) {
            uyaw = -0.8;
        }

        //mensagem para publicacao no pelican
        cmd_accel_msg_quad1.x = -accx;
        cmd_accel_msg_quad1.y = accy;
        cmd_accel_msg_quad1.z = accz;
        cmd_accel_msg_quad1.yaw = uyaw;
        cmd_accel_msg_quad1.type = asctec_hl_comm::mav_ctrl::acceleration;
        cmd_accel_msg_quad1.v_max_xy = -1; // use max velocity from config
        cmd_accel_msg_quad1.v_max_z = -1;
        cmd_accel_pub_quad1.publish(cmd_accel_msg_quad1);

        loop_rate.sleep();
    }
    return 0;
}
