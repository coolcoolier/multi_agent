#include "ros/ros.h"
#include "multi_agent/agent_task_1.h"
#include "multi_agent/agent_task_2.h"
#include "multi_agent/robot_state.h"
#include "iostream"
#include "stack"
#include "boost/thread.hpp"
#include "vector"

using namespace std;

enum state
{
  executing=0,
  ready
};

class factory_server
{
public:
  factory_server();
  ~factory_server();
  void agent_state(const multi_agent::robot_state::ConstPtr& state);
  void send_task1();
  void send_task2();

private:
  ros::NodeHandle nh_;
  ros::ServiceClient client1_;
  ros::ServiceClient client2_;
  ros::Subscriber sub_;
  boost::mutex mutex_1_;
  boost::mutex mutex_2_;
  boost::mutex mutex_task_;
  boost::thread* send_task_thread_1_;
  boost::thread* send_task_thread_2_;
  stack<int> task_;
  vector<int> completed_task_;
  int robot1_state_;
  int robot2_state_;
};

factory_server::factory_server()
{
  client1_ = nh_.serviceClient<multi_agent::agent_task_1>("agent_task_1");
  client2_ = nh_.serviceClient<multi_agent::agent_task_1>("agent_task_2");
  robot1_state_=executing;
  robot2_state_=executing;
  for(int i=5;i>0;i--)
    task_.push(i);

  sub_ = nh_.subscribe("/agent_feedback", 1000, &factory_server::agent_state,this);
  send_task_thread_1_ = new boost::thread(boost::bind(&factory_server::send_task1, this));
  send_task_thread_2_ = new boost::thread(boost::bind(&factory_server::send_task2, this));
}

factory_server::~factory_server()
{
  if(send_task_thread_1_)
  {
    send_task_thread_1_->join();
    delete send_task_thread_1_;
  }
  if(send_task_thread_2_)
  {
    send_task_thread_2_->join();
    delete send_task_thread_2_;
  }

}

void factory_server::send_task1()
{
  while(ros::ok())
  {
    mutex_1_.lock();

    if(robot1_state_ && !task_.empty())
    {
      mutex_1_.unlock();
      multi_agent::agent_task_1 srv;
      srv.request.unfinished_task=task_.top();
      task_.pop();
      if (client1_.call(srv))
      {
        mutex_task_.lock();
        completed_task_.push_back(srv.response.finish_task);
        mutex_task_.unlock();
      }
      else
      {
        ROS_ERROR("Failed to call service agent_task_1");
      }
    }

    else
      mutex_1_.unlock();

  }
}

void factory_server::send_task2()
{
  while(ros::ok())
  {
    mutex_2_.lock();

    if(robot2_state_ && !task_.empty())
    {
      mutex_2_.unlock();
      multi_agent::agent_task_2 srv;
      srv.request.unfinished_task=task_.top();
      task_.pop();
      if (client2_.call(srv))
      {
        mutex_task_.lock();
        completed_task_.push_back(srv.response.finish_task);
        mutex_task_.unlock();
      }
      else
      {
        ROS_ERROR("Failed to call service agent_task_1");
      }
    }

    else
      mutex_2_.unlock();

  }
}

void factory_server::agent_state(const multi_agent::robot_state::ConstPtr &state)
{
  mutex_1_.lock();
  if(state->robot1_ready)
    robot1_state_=ready;
  else
    robot1_state_=executing;
  mutex_1_.unlock();

  mutex_2_.lock();
  if(state->robot2_ready)
    robot2_state_=ready;
  else
    robot2_state_=executing;
  mutex_2_.unlock();

  boost::mutex::scoped_lock lock(mutex_task_);

  if(completed_task_.empty())
  {
    cout<<"none of tasks have been completed. "<<endl;
    return;
  }

  if(completed_task_.size() == 5)
  {
    cout<<"all tasks have been completed. "<<endl;
    return;
  }

  for(int i=0;i<completed_task_.size();i++)
  {
    cout<<"task "<<completed_task_[i]<<" has been completed. "<<endl;
  }
  cout<<"other tasks not completed. "<<endl;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_mini_factory_server");

  factory_server factory_server1;

  ros::spin();

  return 0;
}
