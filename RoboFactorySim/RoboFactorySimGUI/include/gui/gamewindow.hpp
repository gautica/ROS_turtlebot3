#ifndef GAMEWINDOW_H
#define GAMEWINDOW_H

#include <QMainWindow>
#include "ui_gamewindow.h"
namespace Ui {
class GameWindow;
}

namespace gui {
class GameWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit GameWindow(QWidget *parent = 0);
  ~GameWindow();

private:
  Ui::GameWindow *ui;
};

}


#endif // GAMEWINDOW_H
