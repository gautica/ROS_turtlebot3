#include "../include/gui/kiwidget.hpp"
#include "../include/gui/param.h"
#include <QPixmap>
#include <QTimer>
#include <iostream>
#include <QString>

namespace gui {
KIWidget::KIWidget(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::KIWidget)
{
  ui->setupUi(this);
}

KIWidget::~KIWidget()
{
  delete ui;
}

void KIWidget::update_window()
{
    std::cout << "update KI_Widget" << products.size() << " " << products_copy.size() << "\n";
    ui->curr_ress_image_robot0->setStyleSheet(set_styleSheet(curr_product_robo0));
    ui->curr_ress_image_robot1->setStyleSheet(set_styleSheet(curr_product_robo1));

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

QString KIWidget::set_styleSheet(int ID)
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
      str = "background-color: rgb(128, 128, 128); border-image:url(:/images/blank.png)";
      break;
  }
  return str;
}

}

