#ifndef PLAYWIDGET_H
#define PLAYWIDGET_H

#include <QWidget>
#include "ui_playwidget.h"

namespace Ui {
class PlayWidget;
}

namespace gui {

class PlayWidget : public QWidget
{
  Q_OBJECT

public:
  explicit PlayWidget(QWidget *parent = 0);
  ~PlayWidget();
  void update_window();
private:
  QString set_styleSheet(int ID);
private:
  Ui::PlayWidget *ui;
};
}


#endif // PLAYWIDGET_H
