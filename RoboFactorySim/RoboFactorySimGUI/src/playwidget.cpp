#include "../include/gui/playwidget.hpp"
#include "../include/gui/param.h"

#include <iostream>

namespace gui {

PlayWidget::PlayWidget(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::PlayWidget)
{
  ui->setupUi(this);
}

PlayWidget::~PlayWidget()
{
  delete ui;
}

void PlayWidget::update_window()
{
  std::cout << "update playerWidget\n";

  int index = products.size() - 1 ;
  if (index >= 0) {
    ui->robot0_ress2->setStyleSheet(set_styleSheet(products[index]));
  } else {
    ui->robot0_ress2->setStyleSheet("background-color: rgb(128, 128, 128); border-image:url(:/images/blank.png)");
  }
  index = products.size() - 2 ;
  if (index >= 0) {
    ui->robot0_ress1->setStyleSheet(set_styleSheet(products[index]));
  } else {
    ui->robot0_ress1->setStyleSheet("background-color: rgb(128, 128, 128); border-image:url(:/images/blank.png)");
  }
  index = products.size() - 3 ;
  if (index >= 0 ) {
    ui->robot0_ress0->setStyleSheet(set_styleSheet(products[index]));
  } else {
    ui->robot0_ress0->setStyleSheet("background-color: rgb(128, 128, 128); border-image:url(:/images/blank.png)");
  }

  index = products_copy.size() - 1;
  if (index >= 0) {
    ui->robot1_ress2->setStyleSheet(set_styleSheet(products_copy[index]));
  } else {
    ui->robot1_ress2->setStyleSheet("background-color: rgb(128, 128, 128); border-image:url(:/images/blank.png)");
  }
  index = products_copy.size() - 2 ;
  if (index >= 0) {
    ui->robot1_ress1->setStyleSheet(set_styleSheet(products_copy[index]));
  } else {
    ui->robot1_ress1->setStyleSheet("background-color: rgb(128, 128, 128); border-image:url(:/images/blank.png)");
  }
  index = products_copy.size() - 3 ;
  if (index >= 0 ) {
    ui->robot1_ress0->setStyleSheet(set_styleSheet(products_copy[index]));
  } else {
    ui->robot1_ress0->setStyleSheet("background-color: rgb(128, 128, 128); border-image:url(:/images/blank.png)");
  }
}

QString PlayWidget::set_styleSheet(int ID)
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

