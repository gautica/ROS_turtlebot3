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
namespace global_planner {
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
  MainWindow(QWidget *parent = 0);
  ~MainWindow();
private Q_SLOTS:
  void stop_simulation();
  void start_simulation();
  void open_MapViewer();
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

private:
  void calc_coordinate_matrix(double x, double y, std::pair<int, int> &pixel);
  int calc_distance(std::pair < int, int > &point1, std::pair < int, int > &point2);
  bool find_resource(const std::string &str, std::pair<int, int> &resource_pos);
  bool find_machine(int ID1, int ID2, std::pair<int, int> &machine_pos);
  template<typename T>
  int signur(T num) {
    if (num < 0) return -1;
    else return 1;
  }
private:
  Ui::MainWindow *ui;
  QProcess *processSim;
};

}  // namespace global_planner_qt

#endif // global_planner_qt_MAIN_WINDOW_H
