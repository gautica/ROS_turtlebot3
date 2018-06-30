#ifndef ROUTING_H
#define ROUTING_H

#include <utility>
#include <string>
#include <ros/ros.h>

namespace global_planner {

class Referee;
class Routing
{
public:
  Routing();
  ~Routing();

  void make_plan();
private:
  void create_blue_product();
  void store_blue_product();
  void create_red_product();
  void store_red_product();
  void create_yellow_product();
  void store_yellow_product();
  void create_violet_product();
  void store_violet_product();
  void create_orange_product();
  void store_orange_product();
  void create_green_product();
  void store_green_product();
  void create_black_product();
  void store_black_product();
/**
  void find_white_resource(std::pair<int, int> &resource_pos);
  void find_blue_resource(std::pair<int, int> &resource_pos);
  void find_red_resource(std::pair<int, int> &resource_pos);
  void find_yellow_resource(std::pair<int, int> &resource_pos);
  void find_violet_resource(std::pair<int, int> &resource_pos);
  void find_orange_resource(std::pair<int, int> &resource_pos);
  void find_green_resource(std::pair<int, int> &resource_pos);
  void find_black_resource(std::pair<int, int> &resource_pos);
*/
  void find_blue_machine(std::pair<int, int> &machine_pos, std::pair<int, int> &resource_pos);
  void find_red_machine(std::pair<int, int> &machine_pos, std::pair<int, int> &resource_pos);
  void find_yellow_machine(std::pair<int, int> &machine_pos, std::pair<int, int> &resource_pos);

  void calc_coordinate_matrix(double x, double y, std::pair<int, int> &pixel);
  int calc_distance(std::pair < int, int > &point1, std::pair < int, int > &point2);
  void wait_resource_to_find(const std::string &str, std::pair<int, int> &resource_pos);
  bool find_resource(const std::string &str, std::pair<int, int> &resource_pos, bool block);
  bool find_machine(int ID1, int ID2, std::pair<int, int> &machine_pos, std::pair<int, int> &resource_pos);

  void pub_resource_status(int index);
  template<typename T>
  int signur(T num) {
    if (num < 0) return -1;
    else return 1;
  }

private:
  Referee* refer;
};

}   // namespace global_planner
#endif // ROUTING_H
