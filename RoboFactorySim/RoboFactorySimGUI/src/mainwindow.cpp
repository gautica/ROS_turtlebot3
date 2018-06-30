#include "../include/gui/mainwindow.hpp"
#include "../include/gui/gamewindow.hpp"
#include "../include/gui/assignment.hpp"
#include "../include/gui/param.h"
#include <iostream>
#include <unistd.h>
#include <QProcess>

namespace gui {

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  processSim = new QProcess;

  ui->modus->addItem("KI VS KI");
  ui->modus->addItem("Player VS Player");
  ui->modus->addItem("As Team");
  connect(ui->start, SIGNAL(clicked(bool)), this, SLOT(start_simulation()));
  connect(ui->quit, SIGNAL(clicked(bool)), this, SLOT(stop_simulation()));
  connect(ui->assignments, SIGNAL(clicked(bool)), this, SLOT(create_assignments()));
  connect(ui->modus, SIGNAL(currentIndexChanged(int)), this, SLOT(set_gamemode()));
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::start_simulation()
{
  // open launch file

  //processSim->start("/bin/bash", QStringList() << "-c" << command);
  processSim->start("xterm", QStringList() << "-e" << "roslaunch RoboFactorySimGUI RoboFactoryGUI.launch");

  while (!init_image_robot0 || !init_image_robot1 || !init_image_arena) {
    sleep(1);
  }
  gameWindow = new GameWindow(this);
  gameWindow->setAttribute(Qt::WA_DeleteOnClose);

  gameWindow->show();
}

void MainWindow::stop_simulation()
{
  processSim->close();
  gameWindow->close();
}

void MainWindow::create_assignments()
{
  Assignment* assignment = new Assignment(this);
  assignment->setAttribute(Qt::WA_DeleteOnClose);

  assignment->show();
}

void MainWindow::set_gamemode()
{
  curr_gamemode = ui->modus->currentIndex();
  std::cout << "ui->modus->currentIndex():" << ui->modus->currentIndex() << "\n";
}

}

