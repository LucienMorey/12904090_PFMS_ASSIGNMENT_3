#include "controller.h"

Controller::Controller()
{
}

Controller::~Controller()
{
  // Join threads
  for (auto& t : threads)
  {
    t.join();
  }

  // delete pointers
  delete estimator_;
  delete tracker_;
  delete planner_;
}

void Controller::begin(std::shared_ptr<Simulator> sim)
{
  sim_ = sim.get();
  estimator_ = new Estimator();
  tracker_ = new PurePursuit();
  planner_ = new TimePlanner();

  estimator_->setSimulator(sim_);

  // spawn sim and start planning/control threads
  threads.push_back(sim_->spawn());
  threads.push_back(std::thread(&Controller::controlThread, this));
  threads.push_back(std::thread(&Controller::plannerThread, this));
}

void Controller::plannerThread()
{
  while (true)
  {
    // triangulate bogie locations
    std::vector<Aircraft> bogies = estimator_->getBogies();

    // if bogies found plan else wait till next step
    if (bogies.size() > 0)
    {
      std::vector<Aircraft> planes;

      // determine current state of friendly aircraft
      Aircraft friendly;
      friendly.pose = sim_->getFriendlyPose();
      friendly.linear_velocity = sim_->getFriendlyLinearVelocity();
      // pushback friendly aircraft so it is always first in the vector
      planes.push_back(friendly);

      // pushback all other detected bogies
      for (auto bogie : bogies)
      {
        planes.push_back(bogie);
      }

      // plan between friendly and triangulated bogies
      planner_->plan(planes);
    }

    // slow the thread to see bogie readings
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void Controller::controlThread()
{
  while (true)
  {
    // Feed the watchdog control timer
    Twist_t next_twist;

    // get current planned path
    std::vector<Pose> poses = planner_->getPath();

    // if path contains at least the friendly and one other bogie then track otherwise set minimum velocity and wait
    if (poses.size() > 2)
    {
      // display all predicted bogie positions
      sim_->testPose(poses);

      // track the closest bogie
      next_twist =
          tracker_->track(sim_->getFriendlyPose(), sim_->getFriendlyLinearVelocity(), poses.front(), poses.at(1));
    }
    else
    {
      // minimimum twist to keep the bogie in the air
      next_twist = { 50, 0, 0 };
    }

    // send the twist message to the sim
    sim_->controlFriendly(next_twist.vX, next_twist.vZ);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}