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

void plannerThread(const std::shared_ptr<Simulator>& sim, const std::shared_ptr<Estimator>& estimator);
void controlThread(const std::shared_ptr<Simulator>& sim, const std::shared_ptr<path_tracker>& tracker,
                   const std::shared_ptr<Estimator>& estimator);

std::vector<Pose> poses;
std::mutex mx;

int main(void)
{
  std::vector<std::thread> threads;

  // Create a shared pointer for classes
  std::shared_ptr<Simulator> sim(new Simulator());
  std::shared_ptr<path_tracker> tracker(new PurePursuit());
  std::shared_ptr<Estimator> estimator(new Estimator());

  // set sim in estimator to start estimation
  estimator->setSimulator(sim);

  // spawn sim and start planning/control threads
  threads.push_back(sim->spawn());
  threads.push_back(std::thread(controlThread, sim, tracker, estimator));
  threads.push_back(std::thread(plannerThread, sim, estimator));

  // Join threads and begin!
  for (auto& t : threads)
  {
    t.join();
  }

  return 0;
}

void plannerThread(const std::shared_ptr<Simulator>& sim, const std::shared_ptr<Estimator>& estimator)
{
  while (true)
  {
    // plot current bogie poses from estimator
    std::unique_lock<std::mutex> lock(mx);
    poses.push_back(sim->getFriendlyPose());
    std::vector<Aircraft> bogies = estimator->getBogies();

    for (auto bogie : bogies)
    {
      poses.push_back(bogie.pose);
    }
    sim->testPose(poses);
    lock.unlock();

    if (poses.size() > 1)
    {
      while (1)
        ;
    }
    // slow the thread to see bogie readings
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void controlThread(const std::shared_ptr<Simulator>& sim, const std::shared_ptr<path_tracker>& tracker,
                   const std::shared_ptr<Estimator>& estimator)
{
  Pose init_pose = sim->getFriendlyPose();
  while (true)
  {
    // Feed the watchdog control timer
    Twist_t next_twist;
    std::vector<Pose> poses = { init_pose, { { 1000, 3000 }, 0.0 } };
    sim->testPose(poses);
    std::unique_lock<std::mutex> lock(mx);
    if (poses.size() > 1)
    {
      next_twist =
          tracker->track(sim->getFriendlyPose(), sim->getFriendlyLinearVelocity(), init_pose, { { 1000, 3000 }, 0.0 });
    }
    else
    {
      next_twist = { 50, 0, 0 };
    }
    lock.unlock();
    sim->controlFriendly(next_twist.vX, next_twist.vZ);
    // sim->testPose(std::vector<Pose>(1, { { -2000.0, 100.0 }, 2.0 }));

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}