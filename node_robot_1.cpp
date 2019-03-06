#include "ros/ros.h"
#include "multi_agent/robot_state.h"
#include "multi_agent/agent_task_1.h"
#include "unistd.h"
#include "boost/thread.hpp"

enum state
{
  executing=0,
  ready
};

class robot1_
{
public:
  robot1_();
  ~robot1_();
  bool work(multi_agent::agent_task_1::Request  &req,
            multi_agent::agent_task_1::Response &res);
  void publish_state();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::ServiceServer service_;
  boost::mutex state_mutex_;
  boost::thread* publish_thread_;
  int state_;
};

robot1_::robot1_()
{
  pub_ = nh_.advertise<multi_agent::robot_state>("/agent_feedback", 1);
  state_=ready;
  service_ = nh_.advertiseService("/agent_task_1", &robot1_::work,this);
  publish_thread_=new boost::thread(boost::bind(&robot1_::publish_state, this));
}

robot1_::~robot1_()
{
  if(publish_thread_)
  {
    publish_thread_->join();
    delete publish_thread_;
  }

}

void robot1_::publish_state()
{
  multi_agent::robot_state msg;
  while(ros::ok())
  {
    state_mutex_.lock();
    msg.robot1_ready=state_;
    if(state_ == ready)
      std::cout<<"robot1 state : ready"<<std::endl;
    else
      std::cout<<"robot1 state : executing"<<std::endl;
    state_mutex_.unlock();
    pub_.publish(msg);
  }

  return;
}

bool robot1_::work(multi_agent::agent_task_1::Request  &req,
                   multi_agent::agent_task_1::Response &res)
{
  state_mutex_.lock();
  state_=executing;
  state_mutex_.unlock();
  res.finish_task=req.unfinished_task;
  sleep(5);
  state_mutex_.lock();
  state_=ready;
  state_mutex_.unlock();
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_robot_1");

  robot1_ robot1;

  ros::spin();

  return 0;
}
