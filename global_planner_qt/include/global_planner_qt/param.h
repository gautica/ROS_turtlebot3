#ifndef MUTEX_H
#define MUTEX_H
#include <QMutex>
#include <QWaitCondition>
typedef struct goal
{
  std::pair<int, int> goal_pos;
  double yaw;
} goal_t;
extern QMutex mutex;
extern QWaitCondition condition;
extern bool stopSim;
extern std::vector<std::vector<int> > gridMap;
extern std::vector<std::pair<int, int> > path;
extern std::vector<goal_t> goals;
extern goal_t curr_goal;
extern std::vector<std::vector<int> > roboter_local_field;
extern std::pair<int, int> roboter_pos;
extern bool is_dest_reachable;
extern bool is_job_finished;
extern bool is_roboter_pos_init;
extern bool is_map_init;
extern bool is_path_init;
extern bool is_job_finished;
extern bool request_new_plan;
//extern bool grabbed;
extern bool is_reach_goal;
extern bool is_goal_ready;
extern int ROW;
extern int COL;
extern double mapResolution;
#endif // MUTEX_H
