#include "../include/gui/imageWidget.hpp"
#include "../include/gui/param.h"
#include <QImage>
#include <QLabel>
#include <QHBoxLayout>
#include <QTimer>
#include <iostream>
#include <string>

namespace gui {

ImageDisplay::ImageDisplay(QWidget *parent, int modell_id) :
  QWidget(parent), modell_id(modell_id),
  ui(new Ui::ImageDisplay)
{
  ui->setupUi(this);


  label = new QLabel;
  label->setBackgroundRole(QPalette::Base);
  label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  label->setScaledContents(true);
  QHBoxLayout *layout = new QHBoxLayout();
  layout->addWidget(label);
  setLayout(layout);

  //image = new Image;

  timer = new QTimer;
  timer->setInterval(1);
  connect(timer, SIGNAL(timeout()), this, SLOT(update()));
  timer->start();
}

ImageDisplay::~ImageDisplay()
{
  delete ui;
}

void ImageDisplay::update()
{
  //std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n";
  if (modell_id == 0) {
    currentImage = QImage(robot0_buffer, robot0_width, robot0_height, QImage::Format_RGB888);
  } else if (modell_id == 1) {
    currentImage = QImage(robot1_buffer, robot1_width, robot1_height, QImage::Format_RGB888);
  } else if(modell_id = 2) {
    currentImage = QImage(arena_buffer, arena_width, arena_height, QImage::Format_RGB888);
  } else {
    std::cout << "unvailid robot_id for image viewer\n";
    exit(-1);
  }

  label->setPixmap(QPixmap::fromImage(currentImage));
}
}


