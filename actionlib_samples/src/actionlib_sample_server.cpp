#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_samples/actionlib_sampleAction.h>
#include <std_msgs/Int32.h>

class ActionlibSampleAction
{
protected:

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  actionlib::SimpleActionServer<actionlib_samples::actionlib_sampleAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to publish feedback/result
  actionlib_samples::actionlib_sampleFeedback feedback_;
  actionlib_samples::actionlib_sampleResult result_;
  int32_t goal_;

public:

  ActionlibSampleAction(std::string name) :
    as_(nh_, name, false), //simple creation of as_
    action_name_(name)
  {
    //registering the goal and feedback callbacks
    //don't need to see the goal before accepting it
    as_.registerGoalCallback(boost::bind(&ActionlibSampleAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&ActionlibSampleAction::preemptCB, this));
    //subscribing to some other topic
    sub_=nh_.subscribe("/random_number", 1, &ActionlibSampleAction::analysisCB, this);

    as_.start();
  }

  ~ActionlibSampleAction(void)
  {
  }
  void preemptCB(){
    ROS_INFO("%s: Preempted", action_name_.c_str());
    //setting the state of action to preempted
    as_.setPreempted();
  }   
  //just saving the goal
  void goalCB(){
    goal_ = as_.acceptNewGoal()->input;
  }
  //acccepting some number from additional topic
  void analysisCB(const std_msgs::Int32ConstPtr& msg){
    // make sure that the action hasn't been canceled
    if(!as_.isActive()){
      return;
    }
    //doing some senseless action
    feedback_.feedback=sqrt(pow(msg->data, 3));
    //returning it as a feedback
    as_.publishFeedback(feedback_);
    result_.goal_stamp=goal_;
    result_.output=sqrt(goal_);
    //condition for the result
    if (msg->data < goal_){
      ROS_INFO("%s: Aborted, condition not satisfied", action_name_.c_str());
      as_.setAborted(result_);
    }
    else {
      ROS_INFO("%s: Succeded", action_name_.c_str());
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "actionlib_sample");
  ActionlibSampleAction actionlib_sample(ros::this_node::getName());
  ros::spin();
  return 0;
}
