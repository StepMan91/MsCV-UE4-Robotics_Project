#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <vector>
#include <string>
#include "location_monitor/Obj"
using std::vector;
using std::string;

class Object{
  public:
    Object(sting name, double x, double y)
        :name(name),c(x),y(y) {}
    string name;
    double x;
    double y;
};

class ObjectMonitor {
    Public:
    ObjectMonitor(): objects_(){
        InitObjects();
    }


void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
   double x = msg->pose.pose.position.x;
   double y = msg->pose.pose.position.x;
 ROS_INFO("oui: %f, Non: %f",x ,y);
}

private:
vector<Object> objects_;

ObjectDistance FindClosest(double x; double y) {

}

void InitObjects() {
    objects_.push_back(Object("This is a Cube",0.1,-0.5));
    objects_.push_back(Object("This is a Cone",0.1,-0.5));
    objects_.push_back(Object("This is a Sphere",0.1,-0.5));
    objects_.push_back(Object("This is a Cylinder",0.1,-0.5));
                    }

};

int main(int argc, char** argv)
{
  //Init ROS
 ros::init(argc, argv, "sim_detection");
 ros::NodeHandle nh;

 ObjectMonitor monitor;

 // Creating the  ROS subscriber
 ros::Subscriber sub= nh.subscribe("odom", 10, &ObjectMonitor::OdomCallback, &monitor);
 ros::spin();
 return 0;
 }
