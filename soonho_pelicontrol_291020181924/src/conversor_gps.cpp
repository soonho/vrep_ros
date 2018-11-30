#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <asctec_hl_comm/Wgs84ToEnu.h>
#include <asctec_hl_comm/GpsCustomCartesian.h>

bool have_reference_ = false;
double ref_latitude_ = 0.0;
double ref_longitude_ = 0.0;
double ref_altitude_ = 0.0;
double px4_latitude_ = 0.0;
double px4_longitude_ = 0.0;
double px4_altitude_ = 0.0;
double err_latitude = 0.0;
double err_longitude = 0.0;
double err_altitude = 0.0;

ros::Subscriber px4_gps_sub;
ros::Publisher px4_gps_pub;

asctec_hl_comm::GpsCustomCartesian ret;

void updateGpsPx4(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    asctec_hl_comm::Wgs84ToEnu::Request req;
    asctec_hl_comm::Wgs84ToEnu::Response res;
    
    req.lat = msg->latitude;
    req.lon = msg->longitude;
    req.alt = msg->altitude;
    
    ros::service::call("gps_to_local_enu", req, res);
    
    ret.position.x = res.x;
    ret.position.y = res.y;
    ret.position.z = res.z;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "conversor_gps");
    ros::NodeHandle nh;
    // Wait until GPS reference parameters are initialized.
    // Note: this loop probably does not belong to a constructor, it'd be better placed in some sort
    // of "init()" function
    do {
        ROS_INFO("Waiting for GPS reference parameters...");
        if (nh.getParam("/gps_ref_latitude", ref_latitude_) &&
            nh.getParam("/gps_ref_longitude", ref_longitude_) &&
            nh.getParam("/gps_ref_altitude", ref_altitude_) &&
            nh.getParam("/px4/gps_ref_latitude", px4_latitude_) &&
            nh.getParam("/px4/gps_ref_longitude", px4_longitude_) &&
            nh.getParam("/px4/gps_ref_altitude", px4_altitude_)) {
            
            err_latitude = px4_latitude_- ref_latitude_;
            err_longitude = px4_longitude_- ref_longitude_;
            err_altitude = px4_altitude_- ref_altitude_;
          have_reference_ = true;
        }
        else {
          ROS_INFO("GPS reference not ready yet, use set_gps_reference_node to set it");
          ros::Duration(0.5).sleep(); // sleep for half a second
        }
    } while (!have_reference_);
    ROS_INFO("GPS reference errors initialized correctly %f, %f, %f", err_latitude, err_longitude, err_altitude);

    px4_gps_sub = nh.subscribe("/mavros/global_position/global", 1, updateGpsPx4);
    px4_gps_pub = nh.advertise<asctec_hl_comm::GpsCustomCartesian> ("/px4/gps_position_custom", 1);
  
    ros::spinOnce();
    ros::Rate loop_rate(10);
    
    while(ros::ok())
	{
		ros::spinOnce();
    
        px4_gps_pub.publish(ret);
        
		loop_rate.sleep();
	}
}
