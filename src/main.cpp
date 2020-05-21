/*! @file
 *
 *  @brief Main entry point for assignment 3.
 *
 *  TODO: Add information here
 *
 *  @author Lucien Morey 12904090
 *  @date 24/5/2020
 */
#include <thread>
#include <vector>
#include <iostream>
#include "simulator.h"

#include "pure_pursuit.h"
#include "estimator.h"
#include "trajectory_planner.h"

void plannerThread(const std::shared_ptr<Simulator>& sim, const std::shared_ptr<Estimator>& estimator,
                   const std::shared_ptr<TrajectoryPlanner>& planner);
void controlThread(const std::shared_ptr<Simulator>& sim, const std::shared_ptr<path_tracker>& tracker,
                   const std::shared_ptr<TrajectoryPlanner>& planner);

std::mutex mx;

int main(void)
{
  std::vector<std::thread> threads;

  // Create a shared pointer for classes
  std::shared_ptr<Simulator> sim(new Simulator());
  std::shared_ptr<path_tracker> tracker(new PurePursuit());
  std::shared_ptr<Estimator> estimator(new Estimator());
  std::shared_ptr<TrajectoryPlanner> planner(new TrajectoryPlanner(sim));

  // set sim in estimator to start estimation
  estimator->setSimulator(sim);

  // spawn sim and start planning/control threads
  threads.push_back(sim->spawn());
  threads.push_back(std::thread(controlThread, sim, tracker, planner));
  threads.push_back(std::thread(plannerThread, sim, estimator, planner));

  // Join threads and begin!
  for (auto& t : threads)
  {
    t.join();
  }

  return 0;
}

void plannerThread(const std::shared_ptr<Simulator>& sim, const std::shared_ptr<Estimator>& estimator,
                   const std::shared_ptr<TrajectoryPlanner>& planner)
{
  while (true)
  {
    // plot current bogie poses from estimator
    std::unique_lock<std::mutex> lock(mx);
    // poses.push_back(sim->getFriendlyPose());
    std::vector<Aircraft> bogies = estimator->getBogies();

    if (bogies.size() == 4)
    {
      std::vector<Aircraft> planes;
      Aircraft friendly;
      friendly.pose = sim->getFriendlyPose();
      friendly.linear_velocity = sim->getFriendlyLinearVelocity();
      planes.push_back(friendly);
      std::vector<Pose> poses;
      for (auto bogie : bogies)
      {
        planes.push_back(bogie);
        poses.push_back(bogie.pose);
      }

      // sim->testPose(poses);

      planner->plan(planes);
    }

    lock.unlock();
    // slow the thread to see bogie readings
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void controlThread(const std::shared_ptr<Simulator>& sim, const std::shared_ptr<path_tracker>& tracker,
                   const std::shared_ptr<TrajectoryPlanner>& planner)
{
  while (true)
  {
    // Feed the watchdog control timer
    Twist_t next_twist;
    // std::unique_lock<std::mutex> lock(mx);
    if (planner->getPath().size() > 1)
    {
      std::vector<Pose> poses = planner->getPath();
      sim->testPose(poses);
      next_twist = tracker->track(sim->getFriendlyPose(), sim->getFriendlyLinearVelocity(), poses.front(), poses.at(1));
    }
    else
    {
      next_twist = { 50, 0, 0 };
    }
    // lock.unlock();
    sim->controlFriendly(next_twist.vX, next_twist.vZ);
    // sim->testPose(std::vector<Pose>(1, { { -2000.0, 100.0 }, 2.0 }));

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}