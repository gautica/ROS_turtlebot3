#ifndef ASSIGNMENT_H
#define ASSIGNMENT_H

#include <QDialog>
#include <ui_assignment.h>

namespace global_planner {

class Assignment : public QDialog
{
  Q_OBJECT

public:
  explicit Assignment(QWidget *parent = 0);
  ~Assignment();

private:
  Ui::Assignment *ui;
};

} // namespace global_planner
#endif // ASSIGNMENT_H
