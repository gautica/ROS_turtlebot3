/**
 * @file /include/global_planner_qt/main_window.hpp
 *
 * @brief Qt based gui for global_planner_qt.
 *
 * @date November 2010
 **/
#ifndef global_planner_qt_MAIN_WINDOW_H
#define global_planner_qt_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include <QObject>

/*****************************************************************************
** Namespace
*****************************************************************************/
class QProcess;
class QKeyEvent;
class QGridLayout;
namespace global_planner {
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class GameWindow : public QMainWindow {
Q_OBJECT

public:
  GameWindow(QWidget *parent = 0);
  ~GameWindow();
  /**
private Q_SLOTS:
  void stop_simulation();
  void start_simulation();
  void open_simulation();
  void quit_simulation();
  void create_assignments();
  void blue_product();
  void rot_product();
  void yellow_product();
  void violet_product();
  void orange_product();
  void black_product();
  void green_product();
  void click_pushButton();
*/
private:
  Ui::MainWindow *ui;
  QProcess *processSim;
  QGridLayout* gridLayout;

};

}  // namespace global_planner_qt

#endif // global_planner_qt_MAIN_WINDOW_H
