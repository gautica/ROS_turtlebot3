#include "../include/global_planner_qt/routing.h"
#include "../include/global_planner_qt/referee.h"
#include "../include/global_planner_qt/param.h"

namespace global_planner {

Routing::Routing()
{
  refer = new Referee;
}

Routing::~Routing()
{
  delete refer;
}

void Routing::make_plan()
{
  switch (curr_product) {
    case BLUE_PRODUCT:
      create_blue_product();
      break;
    case RED_PRODUCT:
      create_red_product();
      break;
    case YELLOW_PRODUCT:
      create_yellow_product();
      break;
    case VIOLET_PRODUCT:

      break;
    case ORANGE_PRODUCT:

      break;
    case GREEN_PRODUCT:

      break;
    case BLACK_PRODUCT:

      break;
    default:
      break;
  }
}

void Routing::create_blue_product()
{

  std::pair<int, int> resource_pos, machine_pos;

  find_white_resource(resource_pos);
  curr_goal.goal_pos = resource_pos;
  curr_goal.yaw = 0;
  curr_goal.task = GRAB;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);
  find_blue_machine(machine_pos, resource_pos);
  curr_goal.goal_pos = machine_pos;
  curr_goal.yaw = 0;
  curr_goal.task = DROP;
  curr_goal.distance_precision = dist_to_machine;
  goals.push(curr_goal);

  curr_goal = goals.front();
  goals.pop();
}

void Routing::create_yellow_product()
{
  std::pair<int, int> resource_pos, machine_pos;

  find_white_resource(resource_pos);
  curr_goal.goal_pos = resource_pos;
  curr_goal.yaw = 0;
  curr_goal.task = GRAB;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);
  find_yellow_machine(machine_pos, resource_pos);
  curr_goal.goal_pos = machine_pos;
  curr_goal.yaw = 0;
  curr_goal.task = DROP;
  curr_goal.distance_precision = dist_to_machine;
  goals.push(curr_goal);

  curr_goal = goals.front();
  goals.pop();
}

void Routing::create_red_product()
{
  std::pair<int, int> resource_pos, machine_pos;

  find_white_resource(resource_pos);
  curr_goal.goal_pos = resource_pos;
  curr_goal.yaw = 0;
  curr_goal.task = GRAB;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);
  find_red_machine(machine_pos, resource_pos);
  curr_goal.goal_pos = machine_pos;
  curr_goal.yaw = 0;
  curr_goal.task = DROP;
  curr_goal.distance_precision = dist_to_machine;
  goals.push(curr_goal);

  curr_goal = goals.front();
  goals.pop();
}

void Routing::create_violet_product()
{

}

void Routing::create_orange_product()
{

}

void Routing::create_green_product()
{

}

void Routing::create_black_product()
{

}

void Routing::find_blue_resource(std::pair<int, int> &resource_pos)
{
  while (!find_resource("Blue", resource_pos)) {
    refer->update_referee_info();
  }
  ROS_ERROR("Blue resource is found");
}

void Routing::find_red_resource(std::pair<int, int> &resource_pos)
{
  while (!find_resource("Red", resource_pos)) {
    refer->update_referee_info();
  }
  ROS_ERROR("Red resource is found");
}

void Routing::find_yellow_resource(std::pair<int, int> &resource_pos)
{
  while (!find_resource("Yellow", resource_pos)) {
    refer->update_referee_info();
  }
  ROS_ERROR("Yellow resource is found");
}

void Routing::find_violet_resource(std::pair<int, int> &resource_pos)
{
  while (!find_resource("Violet", resource_pos)) {
    refer->update_referee_info();
  }
  ROS_ERROR("Violet resource is found");
}

void Routing::find_orange_resource(std::pair<int, int> &resource_pos)
{
  while (!find_resource("Orange", resource_pos)) {
    refer->update_referee_info();
  }
  ROS_ERROR("Orange resource is found");
}

void Routing::find_green_resource(std::pair<int, int> &resource_pos)
{
  while (!find_resource("Green", resource_pos)) {
    refer->update_referee_info();
  }
  ROS_ERROR("Green resource is found");
}

void Routing::find_white_resource(std::pair<int, int> &resource_pos)
{
  while (!find_resource("White", resource_pos)) {
    refer->update_referee_info();
  }
  ROS_ERROR("white resource is found");
}

void Routing::find_blue_machine(std::pair<int, int> &machine_pos, std::pair<int, int> &resource_pos)
{
  while (!find_machine(0, 3, machine_pos, resource_pos)) {
    refer->update_referee_info();
  }
  ROS_ERROR("blue machine is found with x = %d, y = %d", machine_pos.second, machine_pos.first);
}

void Routing::find_red_machine(std::pair<int, int> &machine_pos, std::pair<int, int> &resource_pos)
{
  while (!find_machine(1, 4, machine_pos, resource_pos)) {
    refer->update_referee_info();
  }
  ROS_ERROR("red machine is found with x = %d, y = %d", machine_pos.second, machine_pos.first);
}

void Routing::find_yellow_machine(std::pair<int, int> &machine_pos, std::pair<int, int> &resource_pos)
{
  while (!find_machine(2, 5, machine_pos, resource_pos)) {
    refer->update_referee_info();
  }
  ROS_ERROR("yellow machine is found with x = %d, y = %d", machine_pos.second, machine_pos.first);
}

void Routing::calc_coordinate_matrix(double x, double y, std::pair<int, int> &pixel) {
  int origin_x = COL / 2;
  int origin_y = ROW / 2;

  int i = origin_x + (int) (x / mapResolution);
  int j = origin_y + (int) (y / mapResolution);

  pixel.second = i;
  pixel.first = j;
}

int Routing::calc_distance(std::pair < int, int > &point1, std::pair < int, int > &point2) {
    return (point1.first - point2.first) * (point1.first - point2.first) +
        (point1.second - point2.second) * (point1.second - point2.second);
}

bool Routing::find_resource(const std::string &str, std::pair<int, int> &resource_pos)
{
  int dist = std::numeric_limits<int>::max();
  int index;
  bool isFound = false;
  for (int i = 0; i < resources.size(); i++) {
    if (((std::string) resources[i].name).find(str) != std::string::npos) {   // found
      calc_coordinate_matrix(resources[i].pos_x, resources[i].pos_y, resource_pos);
      int temp_dist = calc_distance(resource_pos, roboter_pos);
      if (temp_dist < dist) {
        dist = temp_dist;
        index = i;
      }
      isFound = true;
    }
  }
  if (isFound) {
    calc_coordinate_matrix(resources[index].pos_x, resources[index].pos_y, resource_pos);
  }

  return isFound;
}

bool Routing::find_machine(int ID1, int ID2, std::pair<int, int> &machine_pos, std::pair<int, int> &resource_pos)
{
  if (machines[ID1].is_working == true && machines[ID2].is_working == true) {
    return false;
  }
  std::pair<int, int> machine1, machine2;
  int dist1, dist2 = std::numeric_limits<int>::max();
  if (machines[ID1].is_working == false) {
    calc_coordinate_matrix(machines[ID1].pos_x, machines[ID1].pos_y, machine1);
    dist1 = calc_distance(resource_pos, machine1);
  }
  if (machines[ID2].is_working == false) {
    calc_coordinate_matrix(machines[ID2].pos_x, machines[ID2].pos_y, machine2);
    dist2 = calc_distance(resource_pos, machine2);
  }

  //machine_pos = (dist1 < dist2) ? machine1 : machine2;
  if (dist1 < dist2) {
    machine_pos.first = machine1.first + 90;
    machine_pos.second = machine1.second;
  } else {
    machine_pos.first = machine2.first + 90;
    machine_pos.second = machine2.second;
  }
  return true;
}

}


