#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "types.h"

class PurePursuit
{
private:
  /* data */
  GlobalOrd target_position_;

  unsigned int max_g_;

  double max_linear_velocity_;
  double min_linear_velocity_;

  const double LINEAR_TOLERANCE = 200.0;

public:
  PurePursuit(/* args */);
  ~PurePursuit();
};

#endif