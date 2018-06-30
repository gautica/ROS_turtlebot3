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
#include "../include/global_planner_qt/game_window.hpp"
#include "../include/global_planner_qt/assignment.hpp"
#include "../include/global_planner_qt/param.h"
#include "../include/global_planner_qt/imageWidget.hpp"
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
#include <QHBoxLayout>
#include <QScrollArea>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace global_planner {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

GameWindow::GameWindow(QWidget *parent)  : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  QHBoxLayout* robot0_layout = new QHBoxLayout();
  ui->robot0_viewer->setLayout(robot0_layout);
  QHBoxLayout* robot1_layout = new QHBoxLayout();
  ui->robot1_viewer->setLayout(robot1_layout);
  QHBoxLayout* arena_layout = new QHBoxLayout();
  ui->global_viewer->setLayout(arena_layout);

  ImageDisplay* image_viewer_robot0 = new ImageDisplay(0, 0);
  ImageDisplay* image_viewer_robot1 = new ImageDisplay(0, 1);
  ImageDisplay* image_viewer_arena = new ImageDisplay(0, 2);
  robot0_layout->addWidget(image_viewer_robot0);
  robot1_layout->addWidget(image_viewer_robot1);
  arena_layout->addWidget(image_viewer_arena);

  //scrollArea_robot0->setWidget(image_viewer_robot0);
  //scrollArea_robot1->setWidget(image_viewer_robot1);


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
  connect(ui->pushButton, SIGNAL(clicked(bool)), this, SLOT(click_pushButton()));
}

GameWindow::~GameWindow() {
  delete ui;
  delete processSim;
}
/**
void MainWindow::stop_simulation()
{
  stopSim = true;
}

void MainWindow::start_simulation()
{
  stopSim = false;
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

void MainWindow::create_assignments()
{

  Assignment* assignment = new Assignment(this);
  assignment->setAttribute(Qt::WA_DeleteOnClose);

  assignment->show();
}

void MainWindow::blue_product()
{
  products.clear();
  products.push_back(BLUE_PRODUCT);
}

void MainWindow::rot_product()
{
  products.clear();
  products.push_back(RED_PRODUCT);
}

void MainWindow::yellow_product()
{
  products.clear();
  products.push_back(YELLOW_PRODUCT);

}

void MainWindow::violet_product()
{
  products.clear();
  products.push_back(VIOLET_PRODUCT);
}

void MainWindow::orange_product()
{
  products.clear();
  products.push_back(ORANGE_PRODUCT);
}

void MainWindow::black_product()
{
  products.clear();
  products.push_back(BLACK_PRODUCT);
}

void MainWindow::green_product()
{
  products.clear();
  products.push_back(GREEN_PRODUCT);
}

void MainWindow::click_pushButton()
{

  ImageDisplay* display = new ImageDisplay(this);
  display->setAttribute(Qt::WA_DeleteOnClose);
  display->show();
}
*/
}  // namespace global_planner

