#ifndef IMAGEDISPLAY_H
#define IMAGEDISPLAY_H
#include <QWidget>
#include "ui_imageWidget.h"
//#include "image.h"

class QImage;
class QLabel;
class QTimer;

namespace Ui {
class ImageDisplay;
}

namespace gui {
class ImageDisplay : public QWidget
{
  Q_OBJECT

public:
  ImageDisplay(QWidget *parent = 0, int modell_id = -1);
  virtual ~ImageDisplay();

private Q_SLOTS:
  void update();
private:
  Ui::ImageDisplay *ui;
  QImage currentImage;
  QLabel* label;
  QTimer* timer;
  int modell_id;
  //Image* image;
};
}



#endif // IMAGEDISPLAY_H
