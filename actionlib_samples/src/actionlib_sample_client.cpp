#include <actionlib_samples/actionlib_sampleAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<actionlib_samples::actionlib_sampleAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "actionlib_sample_client");
  Client client("actionlib_sample", true); //run client in its own thread
  ROS_INFO("Waiting for action server to start.");
  client.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, sending goal.");
  actionlib_samples::actionlib_sampleGoal goal;// send a goal to the action
  goal.input = 20;
  client.sendGoal(goal);
  //wait for the action to return
  bool finished_before_timeout = client.waitForResult();
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = client.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  //exit
  return 0;
}