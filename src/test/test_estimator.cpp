#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

// Student defined libraries
#include "../estimator.h"

using namespace std;

//==================== UNIT TEST START ====================//
// Test that data measurements are within limits
TEST(DataDistanceTest, Estimator)
{
  Estimator e;

  GlobalOrd test = e.transformBogietoGlobal({ { 1.0, 1.0 }, 0 }, { sqrt(2), (5.0 / 4.0) * M_PI, 10 });
  GlobalOrd expected = { 0.0, 0.0 };
  EXPECT_DOUBLE_EQ(expected.x, test.x);
  EXPECT_DOUBLE_EQ(expected.y, test.y);
}

int main(int argc, char** argv)
{
  shared_ptr<Simulator> sim_(new Simulator());
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
