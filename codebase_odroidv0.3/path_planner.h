#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

class PlannerInputs;
class QR_Commands;

class PathPlanner
{
public:
  PathPlanner();
  void update(PlannerInputs &planner_inputs, QR_Commands &qr_commands);
 
};

#endif
