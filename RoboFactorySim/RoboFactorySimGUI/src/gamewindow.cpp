#include "../include/gui/gamewindow.hpp"
#include "../include/gui/assignment.hpp"
#include "../include/gui/param.h"
#include "../include/gui/imageWidget.hpp"
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

namespace gui {
GameWindow::GameWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::GameWindow)
{
  ui->setupUi(this);

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
}

GameWindow::~GameWindow()
{
  delete ui;
}
}

