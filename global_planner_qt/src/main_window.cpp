/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "../include/global_planner_qt/main_window.hpp"
#include "../include/global_planner_qt/mapviewer.hpp"
#include "../include/global_planner_qt/assignment.hpp"
#include "../include/global_planner_qt/param.h"
#include "../include/global_planner_qt/routing.h"
#include "ui_main_window.h"
#include <QScrollArea>
#include <QLabel>
#include <QImage>
#include <QRgb>
#include <QTimer>
#include <QMessageBox>
#include <QFileDialog>
#include <QProcess>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace global_planner {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(QWidget *parent)  : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  //ui->create_assignment->show();
  processSim = new QProcess(this);
  connect(ui->stop, SIGNAL(clicked(bool)), this, SLOT(stop_simulation()));
  connect(ui->start, SIGNAL(clicked(bool)), this, SLOT(start_simulation()));
  connect(ui->map_viewer, SIGNAL(clicked(bool)), this, SLOT(open_MapViewer()));
  connect(ui->blueButton, SIGNAL(clicked(bool)), this, SLOT(blue_product()));
  connect(ui->rotButton, SIGNAL(clicked(bool)), this, SLOT(rot_product()));
  connect(ui->yellowButton, SIGNAL(clicked(bool)), this, SLOT(yellow_product()));
  connect(ui->violetButton, SIGNAL(clicked(bool)), this, SLOT(violet_product()));
  connect(ui->orangeButton, SIGNAL(clicked(bool)), this, SLOT(orange_product()));
  connect(ui->blackButton, SIGNAL(clicked(bool)), this, SLOT(black_product()));
  connect(ui->greenButton, SIGNAL(clicked(bool)), this, SLOT(green_product()));
  connect(ui->open_simulation_action, SIGNAL(triggered(bool)), this, SLOT(open_simulation()));
  connect(ui->quit_simulation_action, SIGNAL(triggered(bool)), this, SLOT(quit_simulation()));
  connect(ui->createAssignment, SIGNAL(clicked(bool)), this, SLOT(create_assignments()));
}

MainWindow::~MainWindow() {
  delete ui;
  delete processSim;
}

void MainWindow::stop_simulation()
{
  stopSim = true;
}

void MainWindow::start_simulation()
{
  stopSim = false;
  condition.wakeAll();
}

void MainWindow::open_simulation()
{
  QString file = QFileDialog::getOpenFileName(this, tr("Open launch File"), "", tr("*.launch"));
  QString command("roslaunch ");
  command += file;
  std::cout << command.toUtf8().constData() << "\n";
  //processSim->start("/bin/bash", QStringList() << "-c" << command);
  processSim->start("xterm", QStringList() << "-e" << command);
}

void MainWindow::quit_simulation()
{
  processSim->close();
}

void MainWindow::open_MapViewer()
{
  MapViewer* mapViewer = new MapViewer(this);
  mapViewer->setAttribute(Qt::WA_DeleteOnClose);
  mapViewer->show();
}

void MainWindow::create_assignments()
{

  Assignment* assignment = new Assignment(this);
  assignment->setAttribute(Qt::WA_DeleteOnClose);
  assignment->show();

}

void MainWindow::blue_product()
{
  curr_product = BLUE_PRODUCT;
  Routing routing;
  routing.make_plan();
  request_new_plan = true;
  is_job_finished = false;
  is_goal_ready = true;
}

void MainWindow::rot_product()
{
  curr_product = RED_PRODUCT;
  Routing routing;
  routing.make_plan();
  request_new_plan = true;

  is_job_finished = false;
  is_goal_ready = true;
}

void MainWindow::yellow_product()
{
  curr_product = YELLOW_PRODUCT;
  Routing routing;
  routing.make_plan();
  request_new_plan = true;

  is_job_finished = false;
  is_goal_ready = true;
}

void MainWindow::violet_product()
{
  curr_product = VIOLET_PRODUCT;
  Routing routing;
  routing.make_plan();
  is_job_finished = false;
}

void MainWindow::orange_product()
{
  curr_product = ORANGE_PRODUCT;
  Routing routing;
  routing.make_plan();
  is_job_finished = false;
}

void MainWindow::black_product()
{
  curr_product = BLACK_PRODUCT;
  Routing routing;
  routing.make_plan();
  is_job_finished = false;
}

void MainWindow::green_product()
{
  curr_product = GREEN_PRODUCT;
  Routing routing;
  routing.make_plan();
  is_job_finished = false;
}

void MainWindow::calc_coordinate_matrix(double x, double y, std::pair<int, int> &pixel) {
  int origin_x = COL / 2;
  int origin_y = ROW / 2;

  int i = origin_x + (int) (x / mapResolution);
  int j = origin_y + (int) (y / mapResolution);

  pixel.second = i;
  pixel.first = j;
}

int MainWindow::calc_distance(std::pair < int, int > &point1, std::pair < int, int > &point2) {
    return signur(point1.first - point2.first) * (point1.first - point2.first) +
        signur(point1.second - point2.second) * (point1.second - point2.second);
}

bool MainWindow::find_resource(const std::string &str, std::pair<int, int> &resource_pos)
{
  int dist = std::numeric_limits<int>::max();
  int index;
  bool isFound = false;
  for (int i = 0; i < resources.size(); i++) {
    if (((std::string) resources[i].name).find(str) != std::string::npos) {   // found
      calc_coordinate_matrix(resources[i].pos_x, resources[i].pos_y, resource_pos);
      int temp_dist = calc_distance(resource_pos, roboter_pos);
      if (temp_dist < dist) {
        dist = temp_dist;
        index = i;
      }
      isFound = true;
    }
  }
  if (isFound) {
    calc_coordinate_matrix(resources[index].pos_x, resources[index].pos_y, resource_pos);
  }

  return isFound;
}

bool MainWindow::find_machine(int ID1, int ID2, std::pair<int, int> &machine_pos)
{
  bool isFound = false;
  if (machines[ID1].is_working == true && machines[ID2].is_working == true) {
    return false;
  }
  std::pair<int, int> machine1, machine2;
  int dist1, dist2 = std::numeric_limits<int>::max();
  if (machines[ID1].is_working == false) {
    calc_coordinate_matrix(machines[ID1].pos_x, machines[ID1].pos_y, machine1);
    dist1 = calc_distance(roboter_pos, machine1);
  }
  if (machines[ID2].is_working == false) {
    calc_coordinate_matrix(machines[ID2].pos_x, machines[ID2].pos_y, machine2);
    dist2 = calc_distance(roboter_pos, machine2);
  }

  machine_pos = (dist1 < dist2) ? machine1 : machine2;
  return true;
}

}  // namespace global_planner

