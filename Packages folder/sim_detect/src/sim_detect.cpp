#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <vector>
#include <string>
#include "sim_detect/objectdetect.h"
#include <math.h>
#

using std::vector;
using std::string;
using sim_detect::objectdetect;


class Object{
  public:
    Object(string name, double x, double y)
        : name(name),x(x),y(y) {}
    string name;
    double x;
    double y;
};

class ObjectMonitor {


   public:
    ObjectMonitor(const ros::Publisher& object_pub)
        : objects_(), object_pub_(object_pub) {
        InitObjects();
    }


void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
   double x = msg->pose.pose.position.x;
   double y = msg->pose.pose.position.x;
   objectdetect ld = FindClosest(x,y);
   object_pub_.publish(ld);

    if (ld.distance <= 1.1){
   ROS_INFO("Tbot has detected a %s",ld.name.c_str());
    }
   // ROS_INFO(" %s, Dist: %f",ld.name.c_str(),ld.distance);
}

private:
vector<Object> objects_;
ros::Publisher object_pub_;


objectdetect FindClosest(double x, double y) {

objectdetect result;

// in case it did not work
result.distance =-1;
    for (size_t i=0; i < objects_.size(); ++i) {
        const   Object& object = objects_[i];
        double xd = abs(object.x - x);
        double yd = abs(object.y - y);
        double distance = sqrt(xd*xd + yd*yd);

        if (result.distance < 0 || distance < result.distance){
            result.name =  object.name;
            result.distance = distance;
        }
    }
 return result;

}

void InitObjects() {
    objects_.push_back(Object("Box", 1.48,1.53));
    objects_.push_back(Object("Cone",0.95,-1.59));
    objects_.push_back(Object("Sphere",-1.2,1.30));
    objects_.push_back(Object("Cylinder",-1.74,-0.93));
                    }

};

int main(int argc, char** argv)
{

  //Init ROS node
 ros::init(argc, argv, "sim_detect");
 // the ROS node Handle
 ros::NodeHandle nh; 

 // Creating the  ROS publisher
 ros::Publisher object_pub = nh.advertise<objectdetect>("object_detected", 5);

 ObjectMonitor monitor(object_pub);

 // Creating the  ROS subscriber
 ros::Subscriber sub= nh.subscribe("odom", 10, &ObjectMonitor::OdomCallback, &monitor);
 ros::spin();
 return 0;
 }
