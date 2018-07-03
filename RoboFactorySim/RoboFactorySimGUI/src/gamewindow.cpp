#include "../include/gui/gamewindow.hpp"
#include "../include/gui/assignment.hpp"
#include "../include/gui/param.h"
#include "../include/gui/imageWidget.hpp"
#include "../include/gui/teamwidget.hpp"
#include "../include/gui/kiwidget.hpp"
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
#include <QTimer>
#include <unistd.h>


namespace gui {
GameWindow::GameWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::GameWindow)
{
  ui->setupUi(this);
  viewer_id_robot0 = 0;
  viewer_id_robot1 = 1;

  robot0_layout = new QHBoxLayout();
  ui->robot0_viewer->setLayout(robot0_layout);
  robot1_layout = new QHBoxLayout();
  ui->robot1_viewer->setLayout(robot1_layout);

  QHBoxLayout* status_layout = new QHBoxLayout();
  ui->Status->setLayout(status_layout);

  while (!init_image_robot0 || !init_image_robot1 || !init_camera_up_robot0 || !init_camera_up_robot1) {
    sleep(1);
  }
  image_viewer_robot0 = new ImageDisplay(0, 0);
  camera_up_robot0 = new ImageDisplay(0, 2);
  image_viewer_robot1 = new ImageDisplay(0, 1);
  camera_up_robot1 = new ImageDisplay(0, 3);

  robot0_layout->addWidget(image_viewer_robot0);
  robot0_layout->addWidget(camera_up_robot0);
  camera_up_robot0->hide();
  robot1_layout->addWidget(image_viewer_robot1);
  switch (curr_gamemode) {
  case KI_VS_KI:
    statusWidget = new KIWidget();
    break;
  case AS_TEAM:
    statusWidget = new TeamWidget();
    break;
  case PLAYER_VS_PLAYER:

    break;
  default:
    break;
  }
  status_layout->addWidget(statusWidget);
  connect(ui->change_viewer_robot0, SIGNAL(clicked(bool)), this, SLOT(change_viewer_robot0()));
  connect(ui->change_viewer_robot1, SIGNAL(clicked(bool)), this, SLOT(change_viewer_robot1()));
  timer = new QTimer();
  timer->setInterval(500);
}

GameWindow::~GameWindow()
{
  delete ui;
}

void GameWindow::change_viewer_robot0()
{
  if (viewer_id_robot0 == 0) {
    image_viewer_robot0->hide();
    camera_up_robot0->show();
    viewer_id_robot0 = 2;
  } else if(viewer_id_robot0 == 2) {
    camera_up_robot0->hide();
    image_viewer_robot0->show();
    viewer_id_robot0 = 0;
  }
}

void GameWindow::change_viewer_robot1()
{
  if (viewer_id_robot1 == 1) {
    image_viewer_robot1->hide();
    camera_up_robot1->show();
    viewer_id_robot1 = 3;
  } else if(viewer_id_robot1 == 3) {
    camera_up_robot1->hide();
    image_viewer_robot1->show();
    viewer_id_robot1 = 1;
  }
}

void GameWindow::update_window()
{
 statusWidget->update();
}

}

