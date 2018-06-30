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
#include "../include/global_planner_qt/drive.h"
#include "ui_main_window.h"
#include <QScrollArea>
#include <QLabel>
#include <QImage>
#include <QRgb>
#include <QTimer>
#include <QMessageBox>
#include <QFileDialog>
#include <QProcess>
#include <QKeyEvent>

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
  //products.push_back(BLUE_PRODUCT);
  Routing routing;
  routing.make_plan();

}

void MainWindow::rot_product()
{
  curr_product = RED_PRODUCT;
  //products.push_back(RED_PRODUCT);
  Routing routing;
  routing.make_plan();
}

void MainWindow::yellow_product()
{
  curr_product = YELLOW_PRODUCT;
  //products.push_back(YELLOW_PRODUCT);
  Routing routing;
  routing.make_plan();

}

void MainWindow::violet_product()
{
  curr_product = VIOLET_PRODUCT;
  //products.push_back(VIOLET_PRODUCT);
  Routing routing;
  routing.make_plan();
}

void MainWindow::orange_product()
{
  curr_product = ORANGE_PRODUCT;
  //products.push_back(ORANGE_PRODUCT);
  Routing routing;
  routing.make_plan();
}

void MainWindow::black_product()
{
  curr_product = BLACK_PRODUCT;
  //products.push_back(BLACK_PRODUCT);
  Routing routing;
  routing.make_plan();
}

void MainWindow::green_product()
{
  curr_product = GREEN_PRODUCT;
  //products.push_back(GREEN_PRODUCT);
  Routing routing;
  routing.make_plan();
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
  Drive driver(event);
  driver.drive();
  std::cout << "out of keyPressed\n";
}

}  // namespace global_planner

