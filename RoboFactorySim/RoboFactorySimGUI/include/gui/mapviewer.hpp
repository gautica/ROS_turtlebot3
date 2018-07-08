#ifndef MAPVIEWER_H
#define MAPVIEWER_H

#include <QtGui/QMainWindow>
#include <QWidget>

class QImage;
class QTimer;
class QScrollArea;
class QLabel;
class QScrollBar;

namespace gui {
class MapThread;
class MapViewer : public QMainWindow
{
  Q_OBJECT
private:
  QScrollArea *scrollArea;
  QLabel* label;
  QImage image;
  QTimer* timer;
  std::pair<int, int> robot0_pos_gui;
  std::pair<int, int> robot1_pos_gui;
  MapThread* mapThread;
public:
  MapViewer(QWidget *parent = 0);
  ~MapViewer();

  void close();
private:
  void drawPath();
  void drawMap();
  void draw_roboter_pos();
  void init_map();

private Q_SLOTS:
  void update_window();
};

}
#endif // MAPVIEWER_H
