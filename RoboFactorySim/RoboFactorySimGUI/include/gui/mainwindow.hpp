#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ui_mainwindow.h"

namespace Ui {
class MainWindow;
}

class QProcess;
namespace gui {

class GameWindow;
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();
private Q_SLOTS:
  void stop_simulation();
  void start_simulation();
  void create_assignments();
  void set_gamemode();
private:
  Ui::MainWindow *ui;
  QProcess *processSim;
  GameWindow* gameWindow;
};
}


#endif // MAINWINDOW_H
