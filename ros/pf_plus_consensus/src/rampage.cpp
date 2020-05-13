#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "potential_fields.cpp"

// publisher do p3_01
ros::Publisher pub_r1;

// listeners dos outros p3
ros::Subscriber sub_p1;
ros::Subscriber sub_p2;
ros::Subscriber sub_p3;
ros::Subscriber sub_q1;
ros::Subscriber sub_o1;
ros::Subscriber sub_o2;
ros::Subscriber sub_o3;

// campos potenciais objetivo e de aliados/obstaculos
PotentialField goal;
PotentialField pf_p1;
PotentialField pf_p2;
PotentialField pf_p3;
PotentialField pf_r1;
PotentialField pf_q1;
PotentialField pf_o1;
PotentialField pf_o2;
PotentialField pf_o3;
PotentialField pf_o4;
PotentialField pf_o5;

// objetos para publicacao
nav_msgs::Odometry retorno;
int sequencer = 0;

//parametros para ganhos
float gain_x = 0.15;
float gain_y = 0.15;
float gain_z = 0.05;
float gain_yaw = 0.3;
float gain_vx = 0.24;
float gain_vy = 0.24;
float gain_vz = 0.15;
float gain_vyaw = 0.15;
float gain_pf = 1.0;
float gain_con = 1.0;
float target_z = 1.0;

//parametros para limitadores e offsets
float max_accxy = 0.3;
float max_accz = 0.5;
float min_accz = 0.465;
float max_uyaw = 0.8;
float offset_z = 0.465;
float offset_yaw = 0.5;

int method = 0;

//inicializacao dos objetos

void initParams() {
}

void initTrees() {
    pf_o1.x = -3.07;
    pf_o1.y = -0.67;
    pf_o1.gain = 1.0;
    pf_o1.radius = 0.3;
    pf_o1.spread = 0.4;

    pf_o2.x = -1.95;
    pf_o2.y = 2.21;
    pf_o2.gain = 1.0;
    pf_o2.radius = 0.3;
    pf_o2.spread = 0.4;

    pf_o3.x = 0.04;
    pf_o3.y = -1.93;
    pf_o3.gain = 1.0;
    pf_o3.radius = 0.3;
    pf_o3.spread = 0.4;

    pf_o4.x = 0.57;
    pf_o4.y = 0.66;
    pf_o4.gain = 1.0;
    pf_o4.radius = 0.3;
    pf_o4.spread = 0.4;

    pf_o5.x = 3.09;
    pf_o5.y = -0.88;
    pf_o5.gain = 1.0;
    pf_o5.radius = 0.3;
    pf_o5.spread = 0.4;
}

void initNarrow() {
    pf_o1.x = 0.0;
    pf_o1.y = 1.1;
    pf_o1.gain = 1.0;
    pf_o1.radius = 0.3;
    pf_o1.spread = 0.5;

    pf_o2.x = 0.0;
    pf_o2.y = 0.0;
    pf_o2.gain = 1.0;
    pf_o2.radius = 0.3;
    pf_o2.spread = 0.5;

    pf_o3.x = 1.25;
    pf_o3.y = 1.1;
    pf_o3.gain = 1.0;
    pf_o3.radius = 0.3;
    pf_o3.spread = 0.5;

    pf_o4.x = 1.25;
    pf_o4.y = 0.0;
    pf_o4.gain = 1.0;
    pf_o4.radius = 0.3;
    pf_o4.spread = 0.5;
}

void p1_Callback(const nav_msgs::Odometry::ConstPtr& msg) {
    pf_p1.x = (double) msg->pose.pose.position.x;
    pf_p1.y = (double) msg->pose.pose.position.y;
    pf_p1.z = (double) msg->pose.pose.position.z;
    pf_p1.vx = (double) msg->twist.twist.linear.x;
    pf_p1.vy = (double) msg->twist.twist.linear.y;
    pf_p1.vz = (double) msg->twist.twist.linear.z;
}

void p2_Callback(const nav_msgs::Odometry::ConstPtr& msg) {
    pf_p2.x = (double) msg->pose.pose.position.x;
    pf_p2.y = (double) msg->pose.pose.position.y;
    pf_p2.z = (double) msg->pose.pose.position.z;
    pf_p2.vx = (double) msg->twist.twist.linear.x;
    pf_p2.vy = (double) msg->twist.twist.linear.y;
    pf_p2.vz = (double) msg->twist.twist.linear.z;
}

void p3_Callback(const nav_msgs::Odometry::ConstPtr& msg) {
    pf_p3.x = (double) msg->pose.pose.position.x;
    pf_p3.y = (double) msg->pose.pose.position.y;
    pf_p3.z = (double) msg->pose.pose.position.z;
    pf_p3.vx = (double) msg->twist.twist.linear.x;
    pf_p3.vy = (double) msg->twist.twist.linear.y;
    pf_p3.vz = (double) msg->twist.twist.linear.z;
}

void q1_Callback(const nav_msgs::Odometry::ConstPtr& msg) {
    pf_q1.x = (double) msg->pose.pose.position.x;
    pf_q1.y = (double) msg->pose.pose.position.y;
    pf_q1.z = (double) msg->pose.pose.position.z;
    pf_q1.vx = (double) msg->twist.twist.linear.x;
    pf_q1.vy = (double) msg->twist.twist.linear.y;
    pf_q1.vz = (double) msg->twist.twist.linear.z;
}

void o1_Callback(const nav_msgs::Odometry::ConstPtr& msg) {
    pf_o1.x = (double) msg->pose.pose.position.x;
    pf_o1.y = (double) msg->pose.pose.position.y;
    pf_o1.z = (double) msg->pose.pose.position.z;
}

void o2_Callback(const nav_msgs::Odometry::ConstPtr& msg) {
    pf_o2.x = (double) msg->pose.pose.position.x;
    pf_o2.y = (double) msg->pose.pose.position.y;
    pf_o2.z = (double) msg->pose.pose.position.z;
}

void o3_Callback(const nav_msgs::Odometry::ConstPtr& msg) {
    pf_o3.x = (double) msg->pose.pose.position.x;
    pf_o3.y = (double) msg->pose.pose.position.y;
    pf_o3.z = (double) msg->pose.pose.position.z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "first_blood");
    ros::NodeHandle n;

    // iniciando subscribers
    sub_p1 = n.subscribe("/odom_p1", 10, p1_Callback);
    sub_p2 = n.subscribe("/odom_p2", 10, p2_Callback);
    sub_p3 = n.subscribe("/odom_p3", 10, p3_Callback);
    sub_q1 = n.subscribe("/odom_q1", 10, q1_Callback);
    sub_o1 = n.subscribe("/obst_01", 10, o1_Callback);
    sub_o2 = n.subscribe("/obst_02", 10, o2_Callback);
    sub_o3 = n.subscribe("/obst_03", 10, o3_Callback);
    ROS_INFO("Control for Neo_The_Chosen_One: online. Rampage.");

    // inicializando advertisers
    pub_r1 = n.advertise<nav_msgs::Odometry>("/odom_r1", 10);

    initParams();
    //    initTrees();
    initNarrow();

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        ros::spinOnce();

        pf_r1.x = 0;
        pf_r1.y = 0;
        pf_r1.z = 0;

        //        pf_r1.x = (pf_p1.x - 0.5 + pf_p2.x + 0.5 + pf_p3.x + 0.5) / 3;
        //        pf_r1.y = (pf_p1.y - 0.0 + pf_p2.y + 0.5 + pf_p3.y + 0.5) / 3;
        //        pf_r1.z = 0;
        pf_r1.x = pf_p1.x;
        pf_r1.y = pf_p1.y;
        pf_r1.z = 0;

        //coleta dos parametros do ROS
        n.getParam("/pf/goal_x", goal.x);
        n.getParam("/pf/goal_y", goal.y);
        n.getParam("/pf/goal_gain", goal.gain);
        n.getParam("/pf/goal_radius", goal.radius);
        n.getParam("/pf/goal_spread", goal.spread);
        n.getParam("/pf/method", method);

        n.getParam("/pf/gain_x", gain_x);
        n.getParam("/pf/gain_y", gain_y);
        n.getParam("/pf/gain_z", gain_z);
        n.getParam("/pf/gain_yaw", gain_yaw);
        n.getParam("/pf/gain_vx", gain_vx);
        n.getParam("/pf/gain_vy", gain_vy);
        n.getParam("/pf/gain_vz", gain_vz);
        n.getParam("/pf/gain_vyaw", gain_vyaw);
        n.getParam("/pf/gain_pf", gain_pf);
        n.getParam("/pf/gain_con", gain_con);

        pf_r1.add(pf_r1.attForce(goal, pf_r1, 10.0));
        pf_r1.saturate(max_accxy);

        PotentialField rep;
        if (method == 3) {
            PotentialField q2, q3;
            q2.x = 3.5;
            q2.y = 1.10;
            q3.x = -3.5;
            q3.y = 1.10;
            if (rep.doIntersect(pf_q1, goal, q2, q3)) {
                rep.add(pf_r1.rotateBoxForce(pf_r1, 3.50, 1.00, -3.50, 1.20));
            } else {
                rep.add(pf_r1.boxForce(pf_r1, 3.50, 1.00, -3.50, 1.20));
            }
        }
        rep.saturate(max_accxy);
        //pf_r1.add(pf_r1.repForce(pf_o1, pf_r1));
        //pf_r1.add(pf_r1.repForce(pf_o2, pf_r1));
        //pf_r1.add(pf_r1.repForce(pf_o3, pf_r1));
        //pf_r1.add(pf_r1.repForce(pf_o4, pf_r1));
        //pf_r1.add(pf_r1.repForce(pf_o5, pf_r1));

        retorno.child_frame_id = "frame_car";
        retorno.header.seq = sequencer++;
        retorno.header.frame_id = "world";
        retorno.pose.pose.position.x = gain_pf * (pf_r1.x + rep.x);
        retorno.pose.pose.position.y = gain_pf * (pf_r1.y + rep.y);
        retorno.pose.pose.position.z = 0.0;

        pub_r1.publish(retorno);

        loop_rate.sleep();
    }

    return 0;
}
