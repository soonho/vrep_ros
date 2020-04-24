#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <cmath>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "potential_fields.cpp"

using namespace std;

// representacao de campos potenciais

struct pfield {
    double x;
    double y;
    double z;
    double gain;
    double radius;
    double spread;
    double vx;
    double vy;
    double vz;
};

// publisher do p3_01
ros::Publisher pub_p1;

// listeners dos outros p3
ros::Subscriber sub_p1;
ros::Subscriber sub_p2;
ros::Subscriber sub_p3;
ros::Subscriber sub_r1;
ros::Subscriber sub_q1;

// campos potenciais objetivo e de aliados
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

// objeto para publicacao
geometry_msgs::Quaternion retorno;

//parametros para ganhos
float gain_x = 0.5;
float gain_y = 0.5;
float gain_z = 0.05;
float gain_yaw = 0.3;
float gain_vx = 1.0;
float gain_vy = 1.0;
float gain_vz = 0.15;
float gain_vyaw = 0.15;
float target_z = 1.0;
float gain_con = 1.0;
float gain_pf = 1.0;

int method = 0;

//parametros para limitadores e offsets
float max_accxy = 0.3;
float max_accz = 0.5;
float min_accz = 0.465;
float max_uyaw = 0.8;
float offset_z = 0.465;
float offset_yaw = 0.5;

//variaveis para o calculo de saturacao
float rmax = 0, rx = 0, ry = 0, rz = 0, ryaw = 0, pf_p3_yaw = 0;

void initParams() {
    pf_p2.gain = 1.0;
    pf_p2.radius = 1.0;
    pf_p2.spread = 1.0;

    pf_p1.gain = 1.0;
    pf_p1.radius = 1.0;
    pf_p1.spread = 1.0;

    pf_p3.gain = 1.0;
    pf_p3.radius = 1.0;
    pf_p3.spread = 1.0;

    goal.gain = 1.0;
    goal.radius = 0.2;
    goal.spread = 0.5;
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

//leis de controle: omniant1 + omniant2 + quad1 (ganhos dinamicos)

PotentialField consensus() {
    PotentialField temp;
    double ux = gain_x * (0.5 * ((pf_p2.x - 1.0) - (pf_q1.x + 1.0)) + 0.5 * ((pf_p3.x - 1.0) - (pf_q1.x + 1.0))) - gain_vx * (pf_q1.vx);
    double uy = gain_y * (0.5 * ((pf_p2.y - 0.5) - (pf_q1.y - 0.0)) + 0.5 * ((pf_p3.y + 0.5) - (pf_q1.y - 0.0))) - gain_vy * (pf_q1.vy);
    double uz = gain_z * (target_z - pf_q1.z) - gain_vz * (pf_q1.vz);
    //uyaw = gain_yaw * (0.5 * ((omniant1_yaw - 0.0) - (quad1_yaw - 0.0)) + 0.5 * ((omniant2_yaw - 0.0) - (quad1_yaw - 0.0))) - gain_vyaw * (quad1_vyaw);

    temp.x = ux; //ux * cos(pf_p3_yaw) + uy * sin(pf_p3_yaw);
    temp.y = uy; //-ux * sin(pf_p3_yaw) + uy * cos(pf_p3_yaw);
    temp.z = uz;
    return temp;
}

void robot_Callback(const nav_msgs::Odometry::ConstPtr& msg) {
    pf_q1.x = (double) msg->pose.pose.position.x;
    pf_q1.y = (double) msg->pose.pose.position.y;
    pf_q1.z = (double) msg->pose.pose.position.z;
    pf_p3_yaw = msg->pose.pose.orientation.w;

    PotentialField con = consensus();
    con.saturate(max_accxy);

    PotentialField att = pf_q1.attForce(goal, pf_q1, 1.0);
    att.saturate(max_accxy);

    PotentialField rep;
    //para forest
    if (method == 1) {
        rep.add(pf_r1.repForce(pf_o1, pf_q1));
        rep.add(pf_r1.repForce(pf_o2, pf_q1));
        rep.add(pf_r1.repForce(pf_o3, pf_q1));
        rep.add(pf_r1.repForce(pf_o4, pf_q1));
        rep.add(pf_r1.repForce(pf_o5, pf_q1));
    }
    //para wall
    if (method == 3) {
        PotentialField q2, q3;
        q2.x = 2.75;
        q2.y = 1.10;
        q3.x = -3.25;
        q3.y = 1.10;
        if (rep.doIntersect(pf_q1, goal, q2, q3)) {
            rep.add(pf_q1.rotateBoxForce(pf_q1, 3.00, 1.00, -3.00, 1.20));
        } else {
            rep.add(pf_q1.boxForce(pf_q1, 3.00, 1.00, -3.00, 1.20));
        }
    }
    //saturacao
    rep.saturate(max_accxy);

    retorno.x = gain_pf * (att.x + rep.x) + gain_con * con.x;
    retorno.y = gain_pf * (att.y + rep.y) + gain_con * con.y;
    retorno.z = 0.0;
    retorno.w = 0.0;

    pub_p1.publish(retorno);
}

void r1_Callback(const nav_msgs::Odometry::ConstPtr& msg) {
    pf_r1.x = (double) msg->pose.pose.position.x;
    pf_r1.y = (double) msg->pose.pose.position.y;
    pf_r1.z = (double) msg->pose.pose.position.z;
    pf_r1.vx = (double) msg->twist.twist.linear.x;
    pf_r1.vy = (double) msg->twist.twist.linear.y;
    pf_r1.vz = (double) msg->twist.twist.linear.z;
}

void p3_Callback(const nav_msgs::Odometry::ConstPtr& msg) {
    pf_p3.x = (double) msg->pose.pose.position.x;
    pf_p3.y = (double) msg->pose.pose.position.y;
}

void p2_Callback(const nav_msgs::Odometry::ConstPtr& msg) {
    pf_p2.x = (double) msg->pose.pose.position.x;
    pf_p2.y = (double) msg->pose.pose.position.y;
}

void p1_Callback(const nav_msgs::Odometry::ConstPtr& msg) {
    pf_p1.x = (double) msg->pose.pose.position.x;
    pf_p1.y = (double) msg->pose.pose.position.y;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "first_blood");
    ros::NodeHandle n;

    // iniciando parametros
    initParams();
    initTrees();
    //initNarrow();

    //coleta dos parametros do ROS
    n.getParam("/pf/goal_x", goal.x);
    n.getParam("/pf/goal_y", goal.y);
    n.getParam("/pf/goal_gain", goal.gain);
    n.getParam("/pf/goal_radius", goal.radius);
    n.getParam("/pf/goal_spread", goal.spread);
    n.getParam("/pf/method", method);
    n.getParam("/pf/gain_con", gain_con);
    n.getParam("/pf/gain_pf", gain_pf);

    // iniciando subscribers
    sub_p1 = n.subscribe("/odom_p1", 10, p1_Callback);
    sub_p2 = n.subscribe("/odom_p2", 10, p2_Callback);
    sub_p3 = n.subscribe("/odom_p3", 10, p3_Callback);
    sub_q1 = n.subscribe("/odom_q1", 10, robot_Callback);
    sub_r1 = n.subscribe("/odom_r1", 10, r1_Callback);
    ROS_INFO("Control for quad_01: online. Quadra kill.");

    // inicializando advertisers
    pub_p1 = n.advertise<geometry_msgs::Quaternion>("/acc_01", 10);

    ros::spin();

    return 0;
}
