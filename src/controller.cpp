#include "controller.h"

Controller::Controller(std::shared_ptr<Simulator> sim)
{
  sim_ = sim.get();
}

Controller::~Controller()
{
  // Join threads and begin!
  for (auto& t : threads)
  {
    t.join();
  }

  delete estimator_;
  delete tracker_;
  delete planner_;
}

void Controller::begin()
{
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
    // plot current bogie poses from estimator
    std::unique_lock<std::mutex> lock(mx);

    // cond.wait(lock, [this]() {
    //   return ((std::chrono::duration<double>(std::chrono::steady_clock::now() - time_point_last_scan).count() >
    //            trajectory_time));
    // });
    time_point_last_scan = std::chrono::steady_clock::now();
    std::vector<Aircraft> bogies = estimator_->getBogies();

    if (bogies.size() > 0)
    {
      std::vector<Aircraft> planes;
      Aircraft friendly;
      friendly.pose = sim_->getFriendlyPose();
      friendly.linear_velocity = sim_->getFriendlyLinearVelocity();
      planes.push_back(friendly);
      std::vector<Pose> poses;
      for (auto bogie : bogies)
      {
        planes.push_back(bogie);
        poses.push_back(bogie.pose);
      }

      // sim->testPose(poses);

      planner_->plan(planes);
    }

    lock.unlock();
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
    std::unique_lock<std::mutex> lock(mx);
    if (planner_->getPath().size() > 1)
    {
      poses = planner_->getPath();
      sim_->testPose(poses);
      next_twist =
          tracker_->track(sim_->getFriendlyPose(), sim_->getFriendlyLinearVelocity(), poses.front(), poses.at(1));
    }
    else
    {
      next_twist = { 50, 0, 0 };
    }
    lock.unlock();

    sim_->controlFriendly(next_twist.vX, next_twist.vZ);
    cond.notify_one();

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}