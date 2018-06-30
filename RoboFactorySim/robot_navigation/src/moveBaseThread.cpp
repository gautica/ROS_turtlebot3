#include "../include/global_planner_qt/moveBaseThread.h"
#include "../include/global_planner_qt/movebase.h"

namespace global_planner {
MoveBaseThread::MoveBaseThread()
{
  move_base = new MoveBase;
}

MoveBaseThread::~MoveBaseThread(){
  delete move_base;
}

void MoveBaseThread::run()
{
  move_base->start();
}

}   // namespace global_planner
