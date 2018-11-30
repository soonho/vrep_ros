#include <algorithm>
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
ros::Subscriber omniant2;
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

//posicao do omniant2
float omniant2_x   = 0;
float omniant2_y   = 0;
float omniant2_z   = 0;
float omniant2_yaw = 0;

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
float gain_x    = 0.15;
float gain_y    = 0.15;
float gain_z    = 0.05;
float gain_yaw  = 0.3;
float gain_vx   = 0.24;
float gain_vy   = 0.24;
float gain_vz   = 0.15;
float gain_vyaw = 0.15;

//parametros para limitadores e offsets
float max_accxy  = 0.15;
float max_accz   = 0.5;
float min_accz   = 0.465;
float max_uyaw   = 0.8;
float offset_z   = 0.465;
float offset_yaw = 0.5;

//variaveis para o calculo de saturacao
float rmax = 0, rx = 0, ry = 0, rz = 0, ryaw = 0;

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

void get_data_omniant2 (const nav_msgs::Odometry::ConstPtr& msg)
{
    omniant2_x   = msg->twist.twist.linear.x;
    omniant2_y   = msg->twist.twist.linear.y;
    omniant2_z   = msg->twist.twist.linear.z;
    omniant2_yaw = msg->twist.twist.angular.z;
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
    quad1 = nh.subscribe("/qsys_quad1", 1, get_data_quad1);

    //topico para leitura dos dados do omniant1
    omniant1 = nh.subscribe("/qsys_omniant1", 1, get_data_omniant1);

    //topico para leitura dos dados do omniant1
    omniant2 = nh.subscribe("/qsys_omniant2", 1, get_data_omniant2);

    //inicializando parametros de objetivo
    nh.setParam("/soonho/x", 2);
    nh.setParam("/soonho/y", 0);
    nh.setParam("/soonho/z", 0.8);
    nh.setParam("/soonho/yaw", 0);

    //inicializando parametros de ganho
    nh.setParam("/soonho/gain_x", 0.15);
    nh.setParam("/soonho/gain_y", 0.15);
    nh.setParam("/soonho/gain_z", 0.05);
    nh.setParam("/soonho/gain_yaw", 0.3);
    nh.setParam("/soonho/gain_vx", 0.24);
    nh.setParam("/soonho/gain_vy", 0.24);
    nh.setParam("/soonho/gain_vz", 0.15);
    nh.setParam("/soonho/gain_vyaw", 0.15);

    //inicializando parametros de limites/offsets
    nh.setParam("/soonho/max_accxy", 0.15);
    nh.setParam("/soonho/max_accz", 0.5);
    nh.setParam("/soonho/min_accz", 0.467);
    nh.setParam("/soonho/max_uyaw", 0.8);
    nh.setParam("/soonho/offset_z", 0.465);
    nh.setParam("/soonho/offset_yaw", 0.5);

    ros::spinOnce();
    ros::Rate loop_rate(20);

    motorInputCallback(1);

    while(ros::ok()) {
        ros::spinOnce();

        //coleta dos parametros de posicao
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

        //coleta dos parametros de ganho
        if (!nh.getParam("/soonho/gain_x", gain_x)) {
            gain_x = 0.15;
        }
        if (!nh.getParam("/soonho/gain_y", gain_y)) {
            gain_y = 0.15;
        }
        if (!nh.getParam("/soonho/gain_z", gain_z)) {
            gain_z = 0.05;
        }
        if (!nh.getParam("/soonho/gain_yaw", gain_yaw)) {
            gain_yaw = 0.3;
        }
        if (!nh.getParam("/soonho/gain_vx", gain_vx)) {
            gain_vx = 0.24;
        }
        if (!nh.getParam("/soonho/gain_vy", gain_vy)) {
            gain_vy = 0.24;
        }
        if (!nh.getParam("/soonho/gain_vz", gain_vz)) {
            gain_vz = 0.15;
        }
        if (!nh.getParam("/soonho/gain_vyaw", gain_vyaw)) {
            gain_vyaw = 0.15;
        }

        //coleta dos parametros de limites/offsets
        if (!nh.getParam("/soonho/max_accxy", max_accxy)) {
            max_accxy = 0.15;
        }
        if (!nh.getParam("/soonho/max_accz", max_accz)) {
            max_accz = 0.5;
        }
        if (!nh.getParam("/soonho/min_accz", min_accz)) {
            min_accz = 0.465;
        }
        if (!nh.getParam("/soonho/max_uyaw", max_uyaw)) {
            max_uyaw = 0.8;
        }
        if (!nh.getParam("/soonho/offset_z", offset_z)) {
            offset_z = 0.465;
        }
        if (!nh.getParam("/soonho/offset_yaw", offset_yaw)) {
            offset_yaw = 0.5;
        }

        //leis de controle: omniant1 + omniant2 + quad1 (ganhos dinamicos)
        ux   = gain_x   * (0.5 * ((omniant1_x   - 1.0) - (quad1_x   + 1.0)) + 0.5 * ((omniant2_x   - 1.0) - (quad1_x   + 1.0))) - gain_vx   * (quad1_vx);
        uy   = gain_y   * (0.5 * ((omniant1_y   - 1.0) - (quad1_y   - 0.0)) + 0.5 * ((omniant2_y   + 1.0) - (quad1_y   - 0.0))) - gain_vy   * (quad1_vy);
        uz   = gain_z   * (target_z   - quad1_z)                                                                                - gain_vz   * (quad1_vz);
        uyaw = gain_yaw * (0.5 * ((omniant1_yaw - 0.0) - (quad1_yaw - 0.0)) + 0.5 * ((omniant2_yaw - 0.0) - (quad1_yaw - 0.0))) - gain_vyaw * (quad1_vyaw);

        //leis de controle: ganhos dinamicos
        //ux   = gain_x   * (target_x   - quad1_x)   - gain_vx   * (quad1_vx);
        //uy   = gain_y   * (target_y   - quad1_y)   - gain_vy   * (quad1_vy);
        //uz   = gain_z   * (target_z   - quad1_z)   - gain_vz   * (quad1_vz);
        //uyaw = gain_yaw * (target_yaw - quad1_yaw) - gain_vyaw * (quad1_vyaw);

        //leis de controle: ganhos estaticos
        //ux   = 0.5 * (target_x - quad1_x)   - 0.8 * (quad1_vx);
        //uy   = 0.5 * (target_y - quad1_y)   - 0.8 * (quad1_vy);
        //uz   = 1.0 * (target_z - quad1_z)   - 0.5 * (quad1_vz) + 0.465;
        //uyaw = 1.0 * (target_yaw - quad1_yaw) - 0.5 * (quad1_vyaw);

        //leis de controle: omniant1 + quad1
        //ux   = 0.5 * (omniant1_x - quad1_x)   - 0.8 * (quad1_vx) + 0 - 1;
        //uy   = 0.5 * (omniant1_y - quad1_y)   - 0.8 * (quad1_vy) + 0 - 0;
        //uz   = 1.0 * (omniant1_z - quad1_z)   - 0.5 * (quad1_vz) + 1 - 0 + 0.465;
        //uyaw = 1.0 * (target_yaw - quad1_yaw) - 0.5 * (quad1_vyaw);

        accx =  ux * cos(quad1_yaw) + uy * sin(quad1_yaw);
        accy = -ux * sin(quad1_yaw) + uy * cos(quad1_yaw);
        accz =  uz;

        //escala
        accx = accx;
        accy = accy;
        accz = accz;

        //saturacao
        rx = abs(accx / max_accxy);
        ry = abs(accy / max_accxy);
        rz = abs(accz / max_accz);
        ryaw = abs(uyaw / max_uyaw);

        rmax = max(rx, ry);
        rmax = max(rmax, rz);
        rmax = max(rmax, ryaw);

        if (rmax > 1) {
            accx = accx / rmax;
            accy = accy / rmax;
            accz = accz / rmax;
            uyaw = uyaw / rmax;
        }
        accz = accz + offset_z;

/*        if (accx > max_accxy) {
            accx = max_accxy;
        } else if (accx < -max_accxy) {
            accx = -max_accxy;
        }

        if (accy > max_accxy) {
            accy = max_accxy;
        } else if (accy < -max_accxy) {
            accy = -max_accxy;
        }
*/
        if (accz > max_accz) {
            accz = max_accz;
        } else if (accz < min_accz) {
            accz = min_accz;
        }
/*
        if(uyaw > 0) {
            uyaw = uyaw + offset_yaw;
        }
        else if(uyaw < 0) {
            uyaw = uyaw - offset_yaw;
        }

        if (uyaw > max_uyaw) {
            uyaw = max_uyaw;
        } else if (uyaw < -max_uyaw) {
            uyaw = -max_uyaw;
        }
*/

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
