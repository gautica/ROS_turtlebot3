#ifndef MUTEX_H
#define MUTEX_H
#include <QMutex>
#include <QWaitCondition>
#include <queue>
#include "../../../machine_node/src/machine.h"
#include "referee_node/referee_resource.h"

typedef struct goal
{
  int task;
  std::pair<int, int> goal_pos;
  double yaw;
  int distance_precision;
} goal_t;

enum Task
{
  GRAB = 0,
  DROP = 1
};

enum Product
{
  BLUE_PRODUCT = 0,
  YELLOW_PRODUCT = 1,
  RED_PRODUCT = 2,
  VIOLET_PRODUCT = 3,
  ORANGE_PRODUCT = 4,
  GREEN_PRODUCT = 5,
  BLACK_PRODUCT = 6
};

extern QMutex mutex;
extern QWaitCondition condition;
extern bool stopSim;

extern std::vector<std::vector<int> > gridMap;
extern std::vector<std::pair<int, int> > path;
extern std::queue<goal_t> goals;
extern goal_t curr_goal;

extern int curr_product;

extern std::vector<std::vector<int> > roboter_local_field;
extern std::pair<int, int> roboter_pos;

extern bool is_dest_reachable;
extern bool is_job_finished;
extern bool is_roboter_pos_init;
extern bool is_map_init;
extern bool is_path_init;
extern bool is_job_finished;
extern bool request_new_plan;
extern bool grabbed;
extern bool is_reach_goal;
extern bool is_goal_ready;

extern int ROW;
extern int COL;
extern double mapResolution;

extern std::vector<referee::Machine::Machine_Struct> machines;
extern std::vector<referee_node::referee_resource> resources;

extern const int costMap_area;
extern const int dist_to_resource;
extern const int dist_to_machine;

#endif // MUTEX_H
