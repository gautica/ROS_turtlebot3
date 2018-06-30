#ifndef ASSIGNMENT_H
#define ASSIGNMENT_H

#include <QDialog>
#include <ui_assignment.h>

namespace Ui {
class Assignment;
}

namespace gui {
class Assignment : public QDialog
{
  Q_OBJECT

public:
  explicit Assignment(QWidget *parent = 0);
  ~Assignment();
private Q_SLOTS:
  void click_ok();
  void click_cancel();

private:
  void create_products();
public:
  Ui::Assignment *ui;
};
}


#endif // ASSIGNMENT_H
