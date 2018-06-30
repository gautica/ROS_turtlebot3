#include "../include/gui/assignment.hpp"
#include "../include/gui/param.h"
#include <iostream>

namespace gui {
Assignment::Assignment(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::Assignment)
{
  ui->setupUi(this);
  this->setWindowTitle("Assignment");
  connect(ui->ok_button, SIGNAL(clicked(bool)), this, SLOT(click_ok()));
  connect(ui->cancel_button, SIGNAL(clicked(bool)), this, SLOT(click_cancel()));
}

void Assignment::click_ok()
{
  create_products();
  products_copy = products;
  std::cout << "Products is initialized \n";
  close();
}

void Assignment::click_cancel()
{
  close();
}

Assignment::~Assignment()
{
  delete ui;
}

void Assignment::create_products()
{
  products.clear();

  if (ui->blue_checkBox->isChecked()) {
    int count = ui->blue_lineEdit->text().toInt();
    while (count > 0) {
      products.push_back(BLUE_PRODUCT);
      count--;
    }
  }
  if (ui->red_checkBox->isChecked()) {
    int count = ui->red_lineEdit->text().toInt();
    while (count > 0) {
      products.push_back(RED_PRODUCT);
      count--;
    }
  }
  if (ui->yellow_checkBox->isChecked()) {
    int count = ui->yellow_lineEdit->text().toInt();
    while (count > 0) {
      products.push_back(YELLOW_PRODUCT);
      count--;
    }
  }
  if (ui->violet_checkBox->isChecked()) {
    int count = ui->violet_lineEdit->text().toInt();
    while (count > 0) {
      products.push_back(VIOLET_PRODUCT);
      count--;
    }
  }
  if (ui->orange_checkBox->isChecked()) {
    int count = ui->orange_lineEdit->text().toInt();
    while (count > 0) {
      products.push_back(ORANGE_PRODUCT);
      count--;
    }
  }
  if (ui->green_checkBox->isChecked()) {
    int count = ui->green_lineEdit->text().toInt();
    while (count > 0) {
      products.push_back(GREEN_PRODUCT);
      count--;
    }
  }
  if (ui->black_checkBox->isChecked()) {
    int count = ui->black_lineEdit->text().toInt();
    while (count > 0) {
      products.push_back(BLACK_PRODUCT);
      count--;
    }
  }
}
}



