#ifndef KIWIDGET_H
#define KIWIDGET_H

#include <QWidget>
#include "ui_kiwidget.h"

namespace Ui {
class KIWidget;
}

class QString;
class QTimer;
namespace gui {
class KIWidget : public QWidget
{
  Q_OBJECT

public:
  explicit KIWidget(QWidget *parent = 0);
  ~KIWidget();
private Q_SLOTS:
  void update_window();

private:
  QString set_styleSheet(int ID);

private:
  Ui::KIWidget *ui;
  QTimer* timer;
};
}


#endif // KIWIDGET_H
