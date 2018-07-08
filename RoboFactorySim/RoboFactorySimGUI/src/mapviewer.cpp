#include "../include/gui/mapviewer.hpp"
#include "../include/gui/mapthread.h"
#include <QLabel>
#include <QScrollArea>
#include <QImage>
#include <QTimer>
#include <QScrollBar>
#include <iostream>
#include <QWheelEvent>
#include "../include/gui/param.h"

namespace gui {

MapViewer::MapViewer(QWidget *parent) : QMainWindow(parent)
{
  mapThread = new MapThread;
  mapThread->start();
  label = new QLabel;
  label->setBackgroundRole(QPalette::Base);
  label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  label->setScaledContents(true);

  scrollArea = new QScrollArea;
  scrollArea->setBackgroundRole(QPalette::Dark);
  scrollArea->setWidget(label);
  scrollArea->setWidgetResizable(true);

  this->setCentralWidget(scrollArea);
  timer = new QTimer;
  timer->setInterval(10);

  this->resize(QSize(400, 400));
  this->setWindowTitle("MapViewer");

  connect(timer, SIGNAL(timeout()), this, SLOT(update_window()));
  timer->start();

  init_map();
}

MapViewer::~MapViewer()
{
  std::cout << "in MapViewer::~MapViewer() \n";
  is_mapViewr_active = false;
  mapThread->wait();
  delete mapThread;
  delete label;
  delete scrollArea;
  delete timer;
}

void MapViewer::init_map()
{
  std::cout << "update map, path and position\n";
  while (!is_map_init || !is_robot0_pos_init || !is_robot1_pos_init) {
    sleep(1);
  }
  // map and roboter position are already initialized
  image = QImage(QSize(ROW, COL), QImage::Format_ARGB32);
  QRgb value;

  for (int i = 0; i < ROW; i++) {
    for (int j = 0; j < COL; j++) {
      if (gridMap[i][j] == 0) {
        value = qRgb(255, 255, 255);  // white free
      } else if (gridMap[i][j] == -1) {
        value = qRgb(128, 128, 128);  // gray unknow
      } else {
        value = qRgb(0, 0, 0);        // black block
      }
      image.setPixel(QPoint(j, i), value);
    }
  }
  // Init roboter position on map
  this->robot0_pos_gui.first = robot0_pos.first;
  this->robot0_pos_gui.second = robot0_pos.second;
  this->robot1_pos_gui.first = robot1_pos.first;
  this->robot1_pos_gui.second = robot1_pos.second;
  // Draw new position of roboter
  value = qRgb(0, 0, 255);   // blue for roboter
  // Init roboter local field
  int roboter_length = 8;
  roboter_local_field.resize(roboter_length);
  for (int i = 0; i < roboter_local_field.size(); i++) {
      roboter_local_field[i].resize(roboter_length);
  }
  int rows = roboter_local_field.size();
  int cols = roboter_local_field[0].size();
  for (int i = 0; i <= rows; i++) {
    for (int j = 0; j <= cols; j++) {
      image.setPixel(QPoint(robot0_pos_gui.second+i, robot0_pos_gui.first+j), value);
      image.setPixel(QPoint(robot1_pos_gui.second+i, robot1_pos_gui.first+j), value);
    }
  }
  label->setPixmap(QPixmap::fromImage(image));
}

void MapViewer::drawMap()
{
  QRgb value;
  for (int i = 0; i < ROW; i++) {
    for (int j = 0; j < COL; j++) {
      if (gridMap[i][j] == 0) {
        value = qRgb(255, 255, 255);  // white free
      } else if (gridMap[i][j] == -1) {
        value = qRgb(128, 128, 128);  // gray unknow
      } else {
        value = qRgb(0, 0, 0);        // black block
      }
      image.setPixel(QPoint(j, i), value);
    }
  }
}

void MapViewer::drawPath()
{
  QRgb value;
  for (int i = 0; i < path_robot0.size(); i++) {
    value = qRgb(0, 255, 0);
    int row = path_robot0[i].first;
    int col = path_robot0[i].second;
    image.setPixel(QPoint(col, row), value);
  }
  for (int i = 0; i < path_robot1.size(); i++) {
    value = qRgb(0, 255, 0);
    int row = path_robot1[i].first;
    int col = path_robot1[i].second;
    image.setPixel(QPoint(col, row), value);
  }
}

void MapViewer::draw_roboter_pos()
{
  QRgb value = qRgb(255, 255, 255);   // white for delete
  // delete old position of roboter
  int rows = roboter_local_field.size();
  int cols = roboter_local_field[0].size();
  for (int i = 0; i <= rows; i++) {
    for (int j = 0; j <= cols; j++) {
      image.setPixel(QPoint(robot0_pos_gui.second+i, robot0_pos_gui.first+j), value);
      image.setPixel(QPoint(robot1_pos_gui.second+i, robot1_pos_gui.first+j), value);
    }
  }
  // Record roboter position
  this->robot0_pos_gui.first = robot0_pos.first;
  this->robot0_pos_gui.second = robot0_pos.second;
  this->robot1_pos_gui.first = robot1_pos.first;
  this->robot1_pos_gui.second = robot1_pos.second;
  // Draw new position of roboter
  value = qRgb(0, 0, 255);   // blue for roboter
  for (int i = - (rows / 2); i <= rows; i++) {
    for (int j = -(cols / 2); j <= cols; j++) {
      image.setPixel(QPoint(robot0_pos_gui.second+i, robot0_pos_gui.first+j), value);
      image.setPixel(QPoint(robot1_pos_gui.second+i, robot1_pos_gui.first+j), value);
    }
  }
}

void MapViewer::update_window()
{
  drawMap();    // update map
  drawPath();   // update path
  draw_roboter_pos();   // update roboter position
  label->setPixmap(QPixmap::fromImage(image));
}

}   // namespace



