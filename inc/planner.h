#ifndef PLANNER_H
#define PLANNER_H

#include <math.h>
#include "axis_cfg.h"
#include "global_def.h"

double calculate_junction_speed(TrajectorySegment_t *prev,TrajectorySegment_t *curr);
void planner_recalculate();
   
    

#endif // PLANNER_H