#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

#define private public

// Student defined libraries
#include "../estimator.h"
#include "../pure_pursuit.h"
#include "simulator.h"

using namespace std;

//==================== UNIT TEST START ====================//
// Test that data measurements are within limits
TEST(PurePursuitTest, circleIntercept)
{
  PurePursuit pursuit;

  GlobalOrd segment_begin = { 0.0, 5.0 };
  GlobalOrd segment_end = { 1.0, 15.0 };

  GlobalOrd expected = { 0.265, 7.654 };

  GlobalOrd current_pose = { 5, 0 };
  double radius = 9;

  std::vector<GlobalOrd> result = pursuit.lineCircleIntercept(segment_begin, segment_end, current_pose, radius);
  ASSERT_NEAR(expected.x, result.front().x, 0.1);
  ASSERT_NEAR(expected.y, result.front().y, 0.1);
}

TEST(PurePursuitTest, minimumDistance)
{
  PurePursuit pursuit;

  GlobalOrd segment_begin = { 0.0, 5.0 };
  GlobalOrd segment_end = { 1.0, 15.0 };

  GlobalOrd expected = { 0.0, 5.0 };

  GlobalOrd current_pose = { 5, 0 };
  GlobalOrd out = pursuit.closestPointAlongSegmentFromPoint(segment_begin, segment_end, current_pose);

  ASSERT_NEAR(expected.x, out.x, 0.1);
  ASSERT_NEAR(expected.y, out.y, 0.1);
}

TEST(EstimatorTest, TransformBogiesToGlobal)
{
  std::shared_ptr<Simulator> sim(new Simulator());
  // Estimator* e;
  // e = new Estimator();
  // e->setSimulator(sim);

  // delete e;

  ASSERT_EQ(1, 1);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}