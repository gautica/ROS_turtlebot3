#ifndef MUTEX_H
#define MUTEX_H
#include <QMutex>
#include <QWaitCondition>
extern QMutex mutex;
extern QWaitCondition condition;
extern bool quitSim;
#endif // MUTEX_H
