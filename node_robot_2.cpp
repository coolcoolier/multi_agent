#include "ros/ros.h"
#include "multi_agent/robot_state.h"
#include "multi_agent/agent_task_2.h"
#include "unistd.h"
#include "boost/thread.hpp"

enum state
{
  executing=0,
  ready
};

class robot2_
{
public:
  robot2_();
  ~robot2_();
  bool work(multi_agent::agent_task_2::Request  &req,
            multi_agent::agent_task_2::Response &res);
  void publish_state();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::ServiceServer service_;
  boost::mutex state_mutex_;
  boost::thread* publish_thread_;
  int state_;
};

robot2_::robot2_()
{
  pub_ = nh_.advertise<multi_agent::robot_state>("/agent_feedback", 1);
  state_=ready;
  service_ = nh_.advertiseService("/agent_task_2", &robot2_::work,this);
  publish_thread_=new boost::thread(boost::bind(&robot2_::publish_state, this));
}

robot2_::~robot2_()
{
  if(publish_thread_)
  {
    publish_thread_->join();
    delete publish_thread_;
  }

}

void robot2_::publish_state()
{
  multi_agent::robot_state msg;
  while(ros::ok())
  {
    state_mutex_.lock();
    msg.robot2_ready=state_;
    if(state_ == ready)
      std::cout<<"robot2 state : ready"<<std::endl;
    else
      std::cout<<"robot2 state : executing"<<std::endl;
    state_mutex_.unlock();
    pub_.publish(msg);
  }

  return;
}

bool robot2_::work(multi_agent::agent_task_2::Request  &req,
                   multi_agent::agent_task_2::Response &res)
{
  state_mutex_.lock();
  state_=executing;
  state_mutex_.unlock();
  res.finish_task=req.unfinished_task;
  sleep(3);
  state_mutex_.lock();
  state_=ready;
  state_mutex_.unlock();
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_robot_2");

  robot2_ robot2;

  ros::spin();

  return 0;
}
