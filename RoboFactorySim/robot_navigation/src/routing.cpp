#include "../include/global_planner_qt/routing.h"
#include "../include/global_planner_qt/referee.h"
#include "../include/global_planner_qt/param.h"
#include "robot_navigation/Product.h"
#include <ros/ros.h>
#include "robot_navigation/got_resource.h"

namespace global_planner {

Routing::Routing()
{
  refer = new Referee;
  publish_machine = nh.advertise<robot_navigation::got_resource>("/roboter/resource", 1);
}

Routing::~Routing()
{
  delete refer;
}

void Routing::make_plan()
{
  if (curr_product == NO_PRODUCT) {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<robot_navigation::Product>("product_assignment");
    robot_navigation::Product service;
    if (roboter_name == "robot_0") {
      service.request.request = 0;
    } else if (roboter_name == "robot_1") {
      service.request.request = 1;
    } else {
      ROS_ERROR("Unvailid robot name, should be robot_0 or robot_1");
      exit(-1);
    }
    if (client.call(service)) {
      ROS_INFO("Request service sucessfully, receieve product number %ld", service.response.result);
      curr_product = service.response.result;
    }

    if (curr_product == NO_PRODUCT) {
      is_job_finished = true;
      is_goal_ready = false;
      request_new_plan = false;
      return;
    }
  }

  switch (curr_product) {
      case BLUE_PRODUCT:
        if (curr_location == WAREHOUSE) {
          create_blue_product();
        } else if (curr_location == BLUE_MACHINE) {
          store_blue_product();
        } else {
          ROS_ERROR("der Roboter hat sich verlaufen");
        }
        break;
      case RED_PRODUCT:
        if (curr_location == WAREHOUSE) {
          create_red_product();
        } else if (curr_location == RED_MACHINE) {
          store_red_product();
        } else {
          ROS_ERROR("der Roboter hat sich verlaufen");
        }
        break;
      case YELLOW_PRODUCT:
        if (curr_location == WAREHOUSE) {
          create_yellow_product();
        } else if (curr_location == YELLOW_MACHINE) {
          store_yellow_product();
        } else {
          ROS_ERROR("der Roboter hat sich verlaufen");
        }
        break;
      case VIOLET_PRODUCT:
        if (curr_location == WAREHOUSE) {
          create_blue_product();
        }else if (curr_location == BLUE_MACHINE) {
          create_violet_product();
        } else if (curr_location == RED_MACHINE){
          store_violet_product();
        }else {
          ROS_ERROR("der Roboter hat sich verlaufen");
        }
        break;
      case ORANGE_PRODUCT:
        if (curr_location == WAREHOUSE) {
          create_yellow_product();
        } else if (curr_location == YELLOW_MACHINE) {
          create_orange_product();
        } else if (curr_location == RED_MACHINE){
          store_orange_product();
        }else {
          ROS_ERROR("der Roboter hat sich verlaufen");
        }
        break;
      case GREEN_PRODUCT:
        if (curr_location == WAREHOUSE) {
          create_yellow_product();
        } else if (curr_location == YELLOW_MACHINE) {
          create_green_product();
        } else if (curr_location == BLUE_MACHINE){
          store_green_product();
        }else {
          ROS_ERROR("der Roboter hat sich verlaufen");
        }
        break;
      case BLACK_PRODUCT:
        if (curr_location == WAREHOUSE) {
          create_yellow_product();
        } else if (curr_location == YELLOW_MACHINE) {
          create_green_product();
        } else if (curr_location == BLUE_MACHINE) {
          create_black_product();
        } else if (curr_location == RED_MACHINE){
          store_black_product();
        }else {
          ROS_ERROR("der Roboter hat sich verlaufen");
        }
        break;
      default:
        break;
    }
}

void Routing::create_blue_product()
{

  std::pair<int, int> resource_pos, machine_pos;

  will_grab = "White";
  wait_resource_to_find(will_grab, resource_pos);
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
  curr_location = BLUE_MACHINE;
  request_new_plan = true;
  is_job_finished = false;
  is_goal_ready = true;
}

void Routing::store_blue_product()
{
  std::pair<int, int> resource_pos;

  will_grab = "Blue";
  wait_resource_to_find(will_grab, resource_pos);
  curr_goal.goal_pos = resource_pos;
  curr_goal.yaw = 0;
  curr_goal.task = GRAB;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);

  std::pair<int, int> warehouse;
  calc_coordinate_matrix(-0.78, -4.20, warehouse);
  curr_goal.goal_pos = warehouse;
  curr_goal.yaw = 0;
  curr_goal.task = DROP;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);

  curr_goal = goals.front();
  goals.pop();
  curr_location = WAREHOUSE;

  curr_product = NO_PRODUCT;
  request_new_plan = true;
  is_job_finished = false;
  is_goal_ready = true;
}

void Routing::create_yellow_product()
{
  std::pair<int, int> resource_pos, machine_pos;

  will_grab = "White";
  wait_resource_to_find(will_grab, resource_pos);
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
  curr_location = YELLOW_MACHINE;
  request_new_plan = true;
  is_job_finished = false;
  is_goal_ready = true;
}

void Routing::store_yellow_product()
{
  std::pair<int, int> resource_pos;

  will_grab = "Yellow";
  wait_resource_to_find(will_grab, resource_pos);
  curr_goal.goal_pos = resource_pos;
  curr_goal.yaw = 0;
  curr_goal.task = GRAB;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);

  std::pair<int, int> warehouse;
  calc_coordinate_matrix(-0.78, -4.20, warehouse);
  curr_goal.goal_pos = warehouse;
  curr_goal.yaw = 0;
  curr_goal.task = DROP;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);

  curr_goal = goals.front();
  goals.pop();
  curr_location = WAREHOUSE;

  curr_product = NO_PRODUCT;
  request_new_plan = true;
  is_job_finished = false;
  is_goal_ready = true;
}

void Routing::create_red_product()
{
  std::pair<int, int> resource_pos, machine_pos;

  will_grab = "White";
  wait_resource_to_find(will_grab, resource_pos);
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
  curr_location = RED_MACHINE;
  request_new_plan = true;
  is_job_finished = false;
  is_goal_ready = true;
}

void Routing::store_red_product()
{
  std::pair<int, int> resource_pos;

  will_grab = "Red";
  wait_resource_to_find(will_grab, resource_pos);
  curr_goal.goal_pos = resource_pos;
  curr_goal.yaw = 0;
  curr_goal.task = GRAB;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);

  std::pair<int, int> warehouse;
  calc_coordinate_matrix(-0.78, -4.20, warehouse);
  curr_goal.goal_pos = warehouse;
  curr_goal.yaw = 0;
  curr_goal.task = DROP;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);

  curr_goal = goals.front();
  goals.pop();
  curr_location = WAREHOUSE;

  curr_product = NO_PRODUCT;
  request_new_plan = true;
  is_job_finished = false;
  is_goal_ready = true;
}

void Routing::create_violet_product()
{
  std::pair<int, int> resource_pos, machine_pos;

    will_grab = "Blue";
    wait_resource_to_find(will_grab, resource_pos);
    curr_goal.goal_pos = resource_pos;
    curr_goal.yaw = 0;
    curr_goal.task = GRAB;
    curr_goal.distance_precision = dist_to_resource;
    goals.push(curr_goal);

    find_red_machine(machine_pos, resource_pos);
    curr_goal.goal_pos = machine_pos;
    curr_goal.yaw = 0;
    curr_goal.task= DROP;
    curr_goal.distance_precision = dist_to_machine;
    goals.push(curr_goal);

    curr_goal = goals.front();
    goals.pop();
    curr_location = RED_MACHINE;
    request_new_plan = true;
    is_job_finished = false;
    is_goal_ready = true;

}
void Routing::store_violet_product()
{
  std::pair<int, int> resource_pos;

  will_grab = "Violet";
  wait_resource_to_find(will_grab, resource_pos);
  curr_goal.goal_pos = resource_pos;
  curr_goal.yaw = 0;
  curr_goal.task = GRAB;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);

  std::pair<int, int> warehouse;
  calc_coordinate_matrix(-0.78, -4.20, warehouse);
  curr_goal.goal_pos = warehouse;
  curr_goal.yaw = 0;
  curr_goal.task = DROP;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);

  curr_goal = goals.front();
  goals.pop();
  curr_location = WAREHOUSE;

  curr_product = NO_PRODUCT;
  request_new_plan = true;
  is_job_finished = false;
  is_goal_ready = true;
}


void Routing::create_orange_product()
{
  std::pair<int, int> resource_pos, machine_pos;

    will_grab = "Yellow";
    wait_resource_to_find(will_grab, resource_pos);
    curr_goal.goal_pos = resource_pos;
    curr_goal.yaw = 0;
    curr_goal.task = GRAB;
    curr_goal.distance_precision = dist_to_resource;
    goals.push(curr_goal);

    find_red_machine(machine_pos, resource_pos);
    curr_goal.goal_pos = machine_pos;
    curr_goal.yaw = 0;
    curr_goal.task= DROP;
    curr_goal.distance_precision = dist_to_machine;
    goals.push(curr_goal);

    curr_goal = goals.front();
    goals.pop();
    curr_location = RED_MACHINE;
    request_new_plan = true;
    is_job_finished = false;
    is_goal_ready = true;
}

void Routing::store_orange_product()
{
  std::pair<int, int> resource_pos;

  will_grab = "Orange";
  wait_resource_to_find(will_grab, resource_pos);
  curr_goal.goal_pos = resource_pos;
  curr_goal.yaw = 0;
  curr_goal.task = GRAB;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);

  std::pair<int, int> warehouse;
  calc_coordinate_matrix(-0.78, -4.20, warehouse);
  curr_goal.goal_pos = warehouse;
  curr_goal.yaw = 0;
  curr_goal.task = DROP;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);

  curr_goal = goals.front();
  goals.pop();
  curr_location = WAREHOUSE;

  curr_product = NO_PRODUCT;
  request_new_plan = true;
  is_job_finished = false;
  is_goal_ready = true;
}

void Routing::create_green_product()
{
  std::pair<int, int> resource_pos, machine_pos;

    will_grab = "Yellow";
    wait_resource_to_find(will_grab, resource_pos);
    curr_goal.goal_pos = resource_pos;
    curr_goal.yaw = 0;
    curr_goal.task = GRAB;
    curr_goal.distance_precision = dist_to_resource;
    goals.push(curr_goal);

    find_blue_machine(machine_pos, resource_pos);
    curr_goal.goal_pos = machine_pos;
    curr_goal.yaw = 0;
    curr_goal.task= DROP;
    curr_goal.distance_precision = dist_to_machine;
    goals.push(curr_goal);

    curr_goal = goals.front();
    goals.pop();
    curr_location = BLUE_MACHINE;
    request_new_plan = true;
    is_job_finished = false;
    is_goal_ready = true;
}

void Routing::store_green_product()
{
  std::pair<int, int> resource_pos;

  will_grab = "Green";
  wait_resource_to_find(will_grab, resource_pos);
  curr_goal.goal_pos = resource_pos;
  curr_goal.yaw = 0;
  curr_goal.task = GRAB;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);

  std::pair<int, int> warehouse;
  calc_coordinate_matrix(-0.78, -4.20, warehouse);
  curr_goal.goal_pos = warehouse;
  curr_goal.yaw = 0;
  curr_goal.task = DROP;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);

  curr_goal = goals.front();
  goals.pop();
  curr_location = WAREHOUSE;

  curr_product = NO_PRODUCT;
  request_new_plan = true;
  is_job_finished = false;
  is_goal_ready = true;
}
void Routing::create_black_product()
{
  std::pair<int, int> resource_pos, machine_pos;

    will_grab = "Green";
    wait_resource_to_find(will_grab, resource_pos);
    curr_goal.goal_pos = resource_pos;
    curr_goal.yaw = 0;
    curr_goal.task = GRAB;
    curr_goal.distance_precision = dist_to_resource;
    goals.push(curr_goal);

    find_red_machine(machine_pos, resource_pos);
    curr_goal.goal_pos = machine_pos;
    curr_goal.yaw = 0;
    curr_goal.task= DROP;
    curr_goal.distance_precision = dist_to_machine;
    goals.push(curr_goal);

    curr_goal = goals.front();
    goals.pop();
    curr_location = RED_MACHINE;
    request_new_plan = true;
    is_job_finished = false;
    is_goal_ready = true;
}

void Routing::store_black_product()
{
  std::pair<int, int> resource_pos;

  will_grab = "Black";
  wait_resource_to_find(will_grab, resource_pos);
  curr_goal.goal_pos = resource_pos;
  curr_goal.yaw = 0;
  curr_goal.task = GRAB;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);

  std::pair<int, int> warehouse;
  calc_coordinate_matrix(-0.78, -4.20, warehouse);
  curr_goal.goal_pos = warehouse;
  curr_goal.yaw = 0;
  curr_goal.task = DROP;
  curr_goal.distance_precision = dist_to_resource;
  goals.push(curr_goal);

  curr_goal = goals.front();
  goals.pop();
  curr_location = WAREHOUSE;

  curr_product = NO_PRODUCT;
  request_new_plan = true;
  is_job_finished = false;
  is_goal_ready = true;
}

void Routing::find_blue_machine(std::pair<int, int> &machine_pos, std::pair<int, int> &resource_pos)
{
  while (!find_machine(0, 3, machine_pos, resource_pos)) {
    refer->update_referee_info();
    //sleep(5);
  }
  ROS_ERROR("blue machine is found with x = %d, y = %d", machine_pos.second, machine_pos.first);
}

void Routing::find_red_machine(std::pair<int, int> &machine_pos, std::pair<int, int> &resource_pos)
{
  /**
  std::pair<int, int> temp;
  calc_coordinate_matrix(machines[1].pos_x, machines[1].pos_y, temp);
  machine_pos.first = temp.first + 90;
  machine_pos.second = temp.second;
  **/
  while (!find_machine(1, 4, machine_pos, resource_pos)) {
    refer->update_referee_info();
    //sleep(5);
  }
  std::pair<int, int> temp;
  calc_coordinate_matrix(machines[1].pos_x, machines[1].pos_y, temp);
  machine_pos.first = temp.first + 90;
  machine_pos.second = temp.second;
  ROS_ERROR("red machine is found with x = %d, y = %d", machine_pos.second, machine_pos.first);

}

void Routing::find_yellow_machine(std::pair<int, int> &machine_pos, std::pair<int, int> &resource_pos)
{
  while (!find_machine(2, 5, machine_pos, resource_pos)) {
    refer->update_referee_info();
    //sleep(5);
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

void Routing::wait_resource_to_find(const string &str, std::pair<int, int> &resource_pos)
{
  refer->update_referee_info();
  while (!find_resource(str, resource_pos, false)) {
    refer->update_referee_info();
    sleep(2);
  }
  sleep(3);
  refer->update_referee_info();
  find_resource(str, resource_pos, true);
  ROS_ERROR("white resource is found");
}

bool Routing::find_resource(const std::string &str, std::pair<int, int> &resource_pos, bool block)
{
  int dist = std::numeric_limits<int>::max();
  int index;
  bool isFound = false;
  for (int i = 0; i < resources.size(); i++) {
    if (((std::string) resources[i].name).find(str) != std::string::npos && !resources[i].blocked) {   // found
      std::cout << "yan resources[i].name: " << resources[i].name << "\n";
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
    if (block) {
      pub_resource_status(index);
    }
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
    machine_id = machines[ID1].id;
  } else {
    machine_pos.first = machine2.first+ 90;
    machine_pos.second = machine2.second;
    machine_id = machines[ID2].id;
  }
  return true;
}

void Routing::pub_resource_status(int index)
{
  robot_navigation::got_resource resource_msg;
  resource_msg.resource_blocked = true;
  resource_msg.resource_name = resources[index].name;
  //sleep(1);
  publish_machine.publish(resource_msg);
}

}


