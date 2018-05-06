#include "movebase.h"

namespace global_planner {

std::vector<std::vector<int> > MoveBase::grid;
std::vector<std::vector<int> > MoveBase::roboter_local_field;
std::vector<Pair> MoveBase::plan;
Pair MoveBase::roboter_pos;

MoveBase::MoveBase()
{
  sub_odom = nh.subscribe("/odom", 1, &MoveBase::odom_callback, this);
  pub_movement = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  init();
}

void MoveBase::init() {
  // Request dynamic map
  if (!RequestMap::requestMap(this->grid)) {
    ROS_ERROR("Failed to request map !!!");
    exit(1);
  }

  // Waiting bis roboter_pos position initialized
  std::cout << "wait roboter position initialization \n";
  while (this->roboter_pos.first == 0) {
    ros::spinOnce();
  }

  // Set Goal manuel
  goal = std::make_pair(210, 180);

  // Init Plan
  planner.makePlan(roboter_pos, goal, grid, plan);
  create_tunnel();
}

void MoveBase::create_tunnel() {
  // Create a tunnel for Burger Size (L x W x H) = 138mm × 178mm × 192mm
  int roboter_length =  (int) 0.138 / RequestMap::mapResolution + 1;
  int roboter_width = (int) 0.178 / RequestMap::mapResolution + 1;

  for (int i = 0; i < plan.size(); i++) {
    grid[plan[i].first][plan[i].second] = -2;

  }

  // Init roboter local field
  roboter_local_field.resize(2 * roboter_length);
  for (int i = 0; i < roboter_local_field.size(); i++) {
    roboter_local_field[i].resize(2 * roboter_length);
  }
}

void MoveBase::excutePlan() {
  /**
  for (int i = plan.size() - 1; i >= 0; i = i - 5) {
    ros::spinOnce();
    move_step(plan[i]);
  }
  */
  //move_step(plan[plan.size() - 3]);
  geometry_msgs::Twist move;
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    move.linear.x = 0.2; //*  // speed value m/s
    find_nearst_pos();
    // calculate in 10 steps
    int theta_x = plan[plan.size()-10].second - this->roboter_pos.second;
    int theta_y = plan[plan.size()-10].first - this->roboter_pos.first;
    float angle = atan2((double) theta_y, (double) theta_x);

    move.angular.z = 0.5 * (angle - current_pose.theta);
    pub_movement.publish(move);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void MoveBase::odom_callback(const nav_msgs::OdometryConstPtr& msg) {
  // current linear position
  current_pose.x = msg->pose.pose.position.x;
  current_pose.y = msg->pose.pose.position.y;

  // current rotation
  tf::Quaternion quat(msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
  tf::Matrix3x3 matrix(quat);
  double roll, pitch, yaw;
  matrix.getRPY(roll, pitch, yaw);
  current_pose.theta = yaw;

  // initialize roboter_pos position as current position
  calc_coordinate_matrix(current_pose.x, current_pose.y, roboter_pos);
}

void MoveBase::calc_coordiante_map(global_planner::Pair &step, CoPair &coordinate) {
  int origin_x = RequestMap::ROW / 2;
  int origin_y = RequestMap::COL / 2;

  double x = ((double) (step.second - origin_x)) * RequestMap::mapResolution;
  double y = ((double) (step.first - origin_y)) * RequestMap::mapResolution;
  ROS_ERROR("origin_x = %d,  origin_y = %d", origin_x, origin_y);
  ROS_ERROR("x = %f   y = %f", x, y);
  ROS_ERROR("step.first=%d, step.second=%d", step.first, step.second);
  coordinate.first = x;
  coordinate.second = y;
}

void MoveBase::calc_coordinate_matrix(double x, double y, Pair &step) {
  int origin_x = RequestMap::COL / 2;
  int origin_y = RequestMap::ROW / 2;

  int i = origin_x + (int) (x / RequestMap::mapResolution);
  int j = origin_y + (int) (y / RequestMap::mapResolution);

  step.second = i;
  step.first = j;
}

void MoveBase::find_nearst_pos() {
  while (calc_distance(plan[plan.size()-1], roboter_pos) > calc_distance(plan[plan.size()-2], roboter_pos)) {
    plan.pop_back();
  }
}

int MoveBase::calc_distance(Pair point1, Pair point2) {
  return signur(point1.first - point2.first) * (point1.first - point2.first) +
         signur(point1.second - point2.second) * (point1.second - point2.second);
}

} // namespace global_planner


