#include "../include/global_planner_qt/assignment.hpp"

namespace global_planner {

Assignment::Assignment(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::Assignment)
{
  ui->setupUi(this);
  this->setWindowTitle("Assignment");
}

Assignment::~Assignment()
{
  delete ui;
}

} // namespace global_planner

