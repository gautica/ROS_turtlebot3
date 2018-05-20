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
#include "../include/global_planner_qt/param.h"
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
  processSim = new QProcess(this);
  connect(ui->stop, SIGNAL(clicked(bool)), this, SLOT(stop_simulation()));
  connect(ui->start, SIGNAL(clicked(bool)), this, SLOT(start_simulation()));
  connect(ui->map_viewer, SIGNAL(clicked(bool)), this, SLOT(open_MapViewer()));
  connect(ui->open_simulation_action, SIGNAL(triggered(bool)), this, SLOT(open_simulation()));
  connect(ui->quit_simulation_action, SIGNAL(triggered(bool)), this, SLOT(quit_simulation()));
}

MainWindow::~MainWindow() {
  delete ui;
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
  processSim->start("/bin/bash", QStringList() << "-c" << command);
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
}  // namespace global_planner

