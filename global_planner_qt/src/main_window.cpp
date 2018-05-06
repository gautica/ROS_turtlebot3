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
#include "ui_main_window.h"
#include <QScrollArea>
#include <QLabel>
#include <QImage>
#include <QRgb>
#include <QTimer>
#include "movebase.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace global_planner {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(QWidget *parent)  : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  label = new QLabel(this);
  label->setBackgroundRole(QPalette::Base);
  label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  label->setScaledContents(true);

  scrollArea = new QScrollArea;
  scrollArea->setBackgroundRole(QPalette::Dark);
  scrollArea->setWidget(label);
  scrollArea->setWidgetResizable(true);

  this->setCentralWidget(scrollArea);
  this->setWindowTitle("Map Viewer");
  this->resize(QSize(500, 500));

  timer = new QTimer;
  timer->setInterval(10);
  connect(timer, SIGNAL(timeout()), this, SLOT(update_roboter_pos()));
  timer->start();
}

MainWindow::~MainWindow() {
  delete label;
  delete scrollArea;
  delete ui;
}

void MainWindow::drawMap()
{
  int rows = MoveBase::grid.size();
  int cols = MoveBase::grid[0].size();
  image = QImage(QSize(rows, cols), QImage::Format_ARGB32);
  QRgb value;
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {      
      if (MoveBase::grid[i][j] == 0) {
        value = qRgb(255, 255, 255);  // white
      } else if (MoveBase::grid[i][j] == -1) {
        value = qRgb(128, 128, 128);  // gray
      } else {
        value = qRgb(0, 0, 0);        // black
      }
      image.setPixel(QPoint(j, i), value);
    }
  }
  drawPath();
  // Init roboter position on map
  this->roboter_pos.first = MoveBase::roboter_pos.first;
  this->roboter_pos.second = MoveBase::roboter_pos.second;
  // Draw new position of roboter
  value = qRgb(0, 0, 255);   // blue for roboter
  rows = MoveBase::roboter_local_field.size();
  cols = MoveBase::roboter_local_field[0].size();
  for (int i = - (rows / 2); i <= rows; i++) {
    for (int j = -(cols / 2); j <= cols; j++) {
      image.setPixel(QPoint(roboter_pos.second+i, roboter_pos.first+j), value);
    }
  }
  label->setPixmap(QPixmap::fromImage(image));
}

void MainWindow::drawPath()
{
  std::cout << "here in drawPath()" << "\n";
  QRgb value;
  for (int i = 0; i < MoveBase::plan.size(); i++) {
    value = qRgb(255, 0, 0);
    int row = MoveBase::plan[i].first;
    int col = MoveBase::plan[i].second;
    image.setPixel(QPoint(col, row), value);
/**
    // draw left tunnel
    value = qRgb(0, 255, 0);
    row = MoveBase::tunnel_l[i].first;
    col = MoveBase::tunnel_l[i].second;
    image.setPixel(QPoint(col, row), value);
    // draw right tunnel
    row = MoveBase::tunnel_r[i].first;
    col = MoveBase::tunnel_r[i].second;
    image.setPixel(QPoint(col, row), value);
    */
  }
}

void MainWindow::update_roboter_pos()
{
  QRgb value = qRgb(255, 255, 255);   // white for delete
  // delete old position of roboter
  int rows = MoveBase::roboter_local_field.size();
  int cols = MoveBase::roboter_local_field[0].size();
  for (int i = - (rows / 2); i <= rows; i++) {
    for (int j = -(cols / 2); j <= cols; j++) {
      image.setPixel(QPoint(roboter_pos.second+i, roboter_pos.first+j), value);
    }
  }

  // Record roboter position
  this->roboter_pos.first = MoveBase::roboter_pos.first;
  this->roboter_pos.second = MoveBase::roboter_pos.second;

  // Draw new position of roboter
  value = qRgb(0, 0, 255);   // blue for roboter
  for (int i = - (rows / 2); i <= rows; i++) {
    for (int j = -(cols / 2); j <= cols; j++) {
      image.setPixel(QPoint(roboter_pos.second+i, roboter_pos.first+j), value);
    }
  }
  label->setPixmap(QPixmap::fromImage(image));
}

}  // namespace global_planner

