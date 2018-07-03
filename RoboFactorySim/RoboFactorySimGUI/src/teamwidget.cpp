#include "../include/gui/teamwidget.hpp"

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

}

