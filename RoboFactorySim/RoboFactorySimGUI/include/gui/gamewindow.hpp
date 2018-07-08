#ifndef GAMEWINDOW_H
#define GAMEWINDOW_H

#include <QMainWindow>
#include "ui_gamewindow.h"
namespace Ui {
class GameWindow;
}

class QHBoxLayout;
class QWidget;
class QTimer;
namespace gui {
class ImageDisplay;
class KIWidget;
class TeamWidget;
class GameWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit GameWindow(QWidget *parent = 0);
  ~GameWindow();

private Q_SLOTS:
  void change_viewer_robot0();
  void change_viewer_robot1();
  void update_window();
  void open_mapViewer();
private:
  void update_viewer();
  void update_status_window();
  void is_game_finished();
private:
  Ui::GameWindow *ui;
  ImageDisplay* image_viewer_robot0;
  ImageDisplay* image_viewer_robot1;
  ImageDisplay* camera_up_robot0;
  ImageDisplay* camera_up_robot1;
  KIWidget* KI_Widget;
  TeamWidget* teamWidget;
  QHBoxLayout* robot0_layout;
  QHBoxLayout* robot1_layout;
  int viewer_id_robot0;
  int viewer_id_robot1;
  QTimer* timer;
};

}


#endif // GAMEWINDOW_H
