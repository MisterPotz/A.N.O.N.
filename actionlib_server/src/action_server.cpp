#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib_server/MyMsgAction.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "math.h"

// class containing action server methods
class MoveRobotAction
{
protected:

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<actionlib_server::MyMsgAction> action_server; // NodeHandle instance must be created before this line. Otherwise strange error occurs.

  std::string action_name_;

  // create messages that are used to published feedback/result
  actionlib_server::MyMsgFeedback feedback_;
  actionlib_server::MyMsgResult result_;

  // define your subscribers here
  ros::Subscriber gt_pos_sub_;
  geometry_msgs::Pose pos_info_;

  //define your publishers here
  ros::Publisher cmd_vel_pub_;
  ros::Publisher posctrl_pub_;
  ros::Publisher takeoff_pub_;
  std_msgs::Empty lift_;

public:

  MoveRobotAction(std::string name) :
    action_server(nh_, name, boost::bind(&MoveRobotAction::actionCb, this, _1), false),
    action_name_(name)
  {
    initializeSubscribers();
    initializePublishers();
    action_server.start();
  }

  ~MoveRobotAction(void)
  {
  }

  

private:

// initialize subscribers here
void initializeSubscribers(void){
gt_pos_sub_ = nh_.subscribe("/robot/gt_pose", 1, &MoveRobotAction::subscriberCb, this);
ROS_INFO("Subscribers initialized");
}

// initialize publishers here
void initializePublishers(void){
cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
posctrl_pub_ = nh_.advertise<std_msgs::Bool>("/robot/posctrl", 1);
takeoff_pub_ = nh_.advertise<std_msgs::Empty>("/robot/takeoff", 1);
ROS_INFO("Publishers initialized");
}

// callback for our gt_vel subscriber
// it recieves velocity info in the form of Twist message
void subscriberCb(const geometry_msgs::PointConstPtr &info){
  pos_info_.position.x = info->position.x;
  pos_info_.position.y = info->position.y;
  pos_info_.position.z = info->position.z;
}

//helper function to calculate euclidean distance between two 3d points in space
double calDistance(geometry_msgs::Pose current, const actionlib_server::MyMsgActionGoalConstPtr &goal){
  double dist;
  dist = sqrt(pow((goal->x - current.position.x), 2) + pow((goal->y - current.position.y), 2) + pow((goal->z - current.position.z), 2));
  //std::cout<<"dist: " << dist << endl;
  return dist;
}

//main action server callback
void actionCb(const actionlib_server::MyMsgActionGoalConstPtr &goal){
  ros::Rate rate(50);
  bool success = true;
  //do the cool stuff here - i have to move the robot
  geometry_msgs::Twist move;
  move.linear.x = goal->x;
  move.linear.y = goal->y;
  move.linear.z = goal->z;
  takeoff_pub_.publish(lift_); // take 0fff robot - though ideally shouldn't be required. May be some simulation bug
  std_msgs::Bool temp;
  temp.data = true;
  posctrl_pub_.publish(temp); // publish data message to posctrl topic of robot to use position control mode
  std::cout << "Position coordinates received are: \n";
  std::cout << "x: " << goal->x << "\ny: " << goal->y << "\nz: " << goal->z << "\n";

  do{
    cmd_vel_pub_.publish(move);
    feedback_.distance = calDistance(pos_info_, goal);
    action_server.publishFeedback(feedback_); // echo /action_server/feedback
    // take care of preemption here
    if (action_server.isPreemptRequested() || !ros::ok()){
      move.linear.x = pos_info_.position.x;
      move.linear.y = pos_info_.position.y;
      move.linear.z = pos_info_.position.z;
      cmd_vel_pub_.publish(move); // make the drone stop as sooon as the preempt occurs - trick-->sending current pose as cmd_vel
      ROS_INFO("%s: Preemted", action_name_.c_str());
      // set the action state to preempted
      action_server.setPreempted();
      success = false;
      break;
    }

    rate.sleep();

  } 
  while(feedback_.distance > 0.1);


  //check if succeded--yes-->return result_
  if(success){
    result_.status = "Desyination arrived!";
    ROS_INFO("%s: Succeded", action_name_.c_str());
    // set the action state to succeded
    action_server.setSucceeded(result_);
  }

}

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_server");

  MoveRobotAction robot("go_to_point");
  ros::Rate rate(0.5);
  int n = 0;
  while(ros::ok()){
    ros::spinOnce();
    ROS_INFO("%d", n);
    n++;
    rate.sleep();
  }

  return 0;
}