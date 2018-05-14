#include "moveBaseThread.h"
#include "movebase.h"

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
  move_base->excutePlan();
}

}   // namespace global_planner
