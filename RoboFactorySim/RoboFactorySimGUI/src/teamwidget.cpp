#include "../include/gui/teamwidget.hpp"
#include "../include/gui/param.h"
#include <iostream>

namespace gui {
TeamWidget::TeamWidget(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::TeamWidget)
{
  ui->setupUi(this);
}

TeamWidget::~TeamWidget()
{
  delete ui;
}

void TeamWidget::update_window()
{
  std::cout << "update teamWidget\n";
  ui->curr_assigment_image_0->setStyleSheet(set_styleSheet(curr_product_robo0));
  ui->curr_assigment_image_1->setStyleSheet(set_styleSheet(curr_product_robo1));

  int index = products.size() - 1;
  if (index >= 0) {
    ui->ressource3->setStyleSheet(set_styleSheet(index));
  } else {
    ui->ressource3->setStyleSheet("background-color: rgb(128, 128, 128); border-image:url(:/images/blank.png)");
  }
  index = products.size() - 2;
  if (index >= 0) {
    ui->ressource2->setStyleSheet(set_styleSheet(index));
  } else {
    ui->ressource2->setStyleSheet("background-color: rgb(128, 128, 128); border-image:url(:/images/blank.png)");
  }
  index = products.size() - 3;
  if (index >= 0) {
    ui->ressource1->setStyleSheet(set_styleSheet(index));
  } else {
    ui->ressource1->setStyleSheet("background-color: rgb(128, 128, 128); border-image:url(:/images/blank.png)");
  }
  index = products.size() - 4;
  if (index >= 0) {
    ui->ressource0->setStyleSheet(set_styleSheet(index));
  } else {
    ui->ressource0->setStyleSheet("background-color: rgb(128, 128, 128); border-image:url(:/images/blank.png)");
  }
  /**
  int index = (products.size() - 1) >= 0 ? products[products.size() - 1] : -1;
  ui->ressource3->setStyleSheet(set_styleSheet(index));
  index = (products.size() - 2) >= 0 ? products[products.size() - 2] : -1;
  ui->ressource2->setStyleSheet(set_styleSheet(index));
  index = (products.size() - 3) >= 0 ? products[products.size() - 3] : -1;
  ui->ressource1->setStyleSheet(set_styleSheet(index));
  index = (products.size() - 4) >= 0 ? products[products.size() - 4] : -1;
  ui->ressource1->setStyleSheet(set_styleSheet(index));
  */
}

QString TeamWidget::set_styleSheet(int ID)
{
  QString str;
  switch (ID) {
    case 0:
      str = "background-color: rgb(128, 128, 128); border-image:url(:/images/Blue.png)";
      break;
    case 1:
      str = "background-color: rgb(128, 128, 128); border-image:url(:/images/Yellow.png)";
      break;
    case 2:
      str = "background-color: rgb(128, 128, 128); border-image:url(:/images/Red.png)";
      break;
    case 3:
      str = "background-color: rgb(128, 128, 128); border-image:url(:/images/Pink.png)";
      break;
    case 4:
      str = "background-color: rgb(128, 128, 128); border-image:url(:/images/Orange.png)";
      break;
    case 5:
      str = "background-color: rgb(128, 128, 128); border-image:url(:/images/Green.png)";
      break;
    case 6:
      str = "background-color: rgb(128, 128, 128); border-image:url(:/images/Black.png)";
      break;
    default:
      str="background-color: rgb(128, 128, 128); border-image:url(:/images/blank.png)";
      break;
  }
  return str;
}

}

