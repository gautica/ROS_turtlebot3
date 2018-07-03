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
  timer = new QTimer();
  timer->setInterval(500);
  connect(timer, SIGNAL(timeout()), this, SLOT(update_window()));
  timer->start();
}

KIWidget::~KIWidget()
{
  delete ui;
}

void KIWidget::update_window()
{
  if (update_status) {
    std::cout << "update\n";
    ui->curr_ress_image_robot0->setStyleSheet(set_styleSheet(curr_product_robo0));
    ui->curr_ress_image_robot1->setStyleSheet(set_styleSheet(curr_product_robo1));

    int index = (products.size() - 1) >= 0 ? products[products.size() - 1] : -1;
    ui->robot0_ress9->setStyleSheet(set_styleSheet(index));
    index = (products.size() - 2) >= 0 ? products[products.size() - 2] : -1;
    ui->robot0_ress8->setStyleSheet(set_styleSheet(index));
    index = (products.size() - 3) >= 0 ? products[products.size() - 3] : -1;
    ui->robot0_ress7->setStyleSheet(set_styleSheet(index));
    index = (products.size() - 4) >= 0 ? products[products.size() - 4] : -1;
    ui->robot0_ress6->setStyleSheet(set_styleSheet(index));
    index = (products.size() - 5) >= 0 ? products[products.size() - 5] : -1;
    ui->robot0_ress5->setStyleSheet(set_styleSheet(index));
    index = (products.size() - 6) >= 0 ? products[products.size() - 6] : -1;
    ui->robot0_ress4->setStyleSheet(set_styleSheet(index));
    index = (products.size() - 7) >= 0 ? products[products.size() - 7] : -1;
    ui->robot0_ress3->setStyleSheet(set_styleSheet(index));
    index = (products.size() - 8) >= 0 ? products[products.size() - 8] : -1;
    ui->robot0_ress2->setStyleSheet(set_styleSheet(index));
    index = (products.size() - 9) >= 0 ? products[products.size() - 9] : -1;
    ui->robot0_ress1->setStyleSheet(set_styleSheet(index));
    index = (products.size() - 10) >= 0 ? products[products.size() - 10] : -1;
    ui->robot0_ress0->setStyleSheet(set_styleSheet(index));

    index = (products_copy.size() - 1) >= 0 ? products_copy[products_copy.size() - 1] : -1;
    ui->robot1_ress9->setStyleSheet(set_styleSheet(index));
    index = (products_copy.size() - 2) >= 0 ? products_copy[products_copy.size() - 2] : -1;
    ui->robot1_ress8->setStyleSheet(set_styleSheet(index));
    index = (products_copy.size() - 3) >= 0 ? products_copy[products_copy.size() - 3] : -1;
    ui->robot1_ress7->setStyleSheet(set_styleSheet(index));
    index = (products_copy.size() - 4) >= 0 ? products_copy[products_copy.size() - 4] : -1;
    ui->robot1_ress6->setStyleSheet(set_styleSheet(index));
    index = (products_copy.size() - 5) >= 0 ? products_copy[products_copy.size() - 5] : -1;
    ui->robot1_ress5->setStyleSheet(set_styleSheet(index));
    index = (products_copy.size() - 6) >= 0 ? products_copy[products_copy.size() - 6] : -1;
    ui->robot1_ress4->setStyleSheet(set_styleSheet(index));
    index = (products_copy.size() - 7) >= 0 ? products_copy[products_copy.size() - 7] : -1;
    ui->robot1_ress3->setStyleSheet(set_styleSheet(index));
    index = (products_copy.size() - 8) >= 0 ? products_copy[products_copy.size() - 8] : -1;
    ui->robot1_ress2->setStyleSheet(set_styleSheet(index));
    index = (products_copy.size() - 9) >= 0 ? products_copy[products_copy.size() - 9] : -1;
    ui->robot1_ress1->setStyleSheet(set_styleSheet(index));
    index = (products_copy.size() - 10) >= 0 ? products_copy[products_copy.size() - 10] : -1;
    ui->robot1_ress0->setStyleSheet(set_styleSheet(index));

    update_status = false;
  }

}

QString KIWidget::set_styleSheet(int ID)
{
  QString str;
  switch (ID) {
    case 0:
      str = "background-color: rgb(238, 238, 236); border-image:url(:/images/Blue.png)";
      break;
    case 1:
      str = "background-color: rgb(238, 238, 236); border-image:url(:/images/Red.png)";
      break;
    case 2:
      str = "background-color: rgb(238, 238, 236); border-image:url(:/images/Yellow.png)";
      break;
    case 3:
      str = "background-color: rgb(238, 238, 236); border-image:url(:/images/Pink.png)";
      break;
    case 4:
      str = "background-color: rgb(238, 238, 236); border-image:url(:/images/Orange.png)";
      break;
    case 5:
      str = "background-color: rgb(238, 238, 236); border-image:url(:/images/Green.png)";
      break;
    case 6:
      str = "background-color: rgb(238, 238, 236); border-image:url(:/images/Black.png)";
      break;
    default:
      str="";
      break;
  }
  return str;
}

}

