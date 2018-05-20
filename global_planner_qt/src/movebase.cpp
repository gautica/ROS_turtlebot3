#include "../include/global_planner_qt/movebase.h"
#include "../include/global_planner_qt/grab.h"
#include "../include/global_planner_qt/param.h"

namespace global_planner {
MoveBase::MoveBase()
{
  sub_odom = nh.subscribe("/odom", 1, &MoveBase::odom_callback, this);
  sub_laser = nh.subscribe("/scan", 1, &MoveBase::scan_callback, this);
  pub_movement = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  /**
  sub_resource = nh.subscribe("/roboter/resource", 10, &MoveBase::resource_callback, this);
  sub_resource_position = nh.subscribe("/gazebo/model_states", 10, &MoveBase::resource_pose_Callback, this);
  
  publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
  sub = nh.subscribe("/gazebo/link_states", 1, &MoveBase::pose_Callback, this);
  */
  moveDirection = FORWARD;
  rotateDirection = CLOCKWISE;
  speed = SLOW;
  steps = LONG;
  /**
  sub_counter = 0;
  current_publish_id = 0;
  found_ressources = false;
  resources_in_simulation = 3;
  */
  //init();
}

void MoveBase::init() {
  // Waiting bis roboter_pos position initialized
  std::cout << "Waiting for roboter position initialized ...\n";
  while (!is_roboter_pos_init) {
    ros::spinOnce();
  }
  std::cout << "Roboter position is initialized ...\n";

  //calc_coordinate_matrix(-2.6, -2.3, curr_goal.goal_pos);
  //curr_goal.yaw = -1.57075;
  //goals.push_back(curr_goal);
  calc_coordinate_matrix(1.6, 1.6, curr_goal.goal_pos);
  curr_goal.yaw = 0.785;
  goals.push_back(curr_goal);
  calc_coordinate_matrix(-2.6, -2.6, curr_goal.goal_pos);
  curr_goal.yaw = -1.57075;

  // Init roboter local field
  int roboter_length =  8;
  roboter_local_field.resize(roboter_length);
  for (int i = 0; i < roboter_local_field.size(); i++) {
    roboter_local_field[i].resize(roboter_length);
  }
}

void MoveBase::start()
{
  while (!is_map_init) {
    sleep(1);
  }
  init();
  is_goal_ready = true;
  while (!is_path_init) {
    sleep(1);
  }
  excutePlan();
}

void MoveBase::excutePlan() {
  geometry_msgs::Twist twist;
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    while (!stopSim && is_dest_reachable) {
      if (!reach_goal(twist)) {
        mutex.lock();     // lock, because path can be changed in MapThread
        find_nearst_pos();
        rotate(twist);
        move(twist);
        ros::spinOnce();
        mutex.unlock();
        loop_rate.sleep();
      }
    }
    // Wait for starting ...
    mutex.lock();
    pubZeroVel(twist);
    condition.wait(&mutex);
    mutex.unlock();
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
  is_roboter_pos_init = true;
}

void MoveBase::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // Find the closest range between the defined minimum and maximum angles
  int minIndex = (int) ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
  int maxIndex = (int) ceil(msg->angle_max / msg->angle_increment);

  float closest_range;
  if (moveDirection == FORWARD) {
    closest_range = msg->ranges[0];
    for (int i = 0; i < minIndex; i++)
    {
      if (msg->ranges[i] < closest_range)
      {
        closest_range = msg->ranges[i];
      }

      if (msg->ranges[maxIndex - i - 2] < closest_range) {
        closest_range = msg->ranges[maxIndex - i - 2];
      }
    }
  } else {
    closest_range = msg->ranges[maxIndex/2 - minIndex];
    for (int i = maxIndex/2 - minIndex; i <= maxIndex/2 + 2*minIndex; i++) {
      if (msg->ranges[i] < closest_range)
      {
        closest_range = msg->ranges[i];
      }
    }
  }

  //ROS_INFO_STREAM("Closest Range: "  << closest_range);
  /**
  if (closest_range < MIN_PROXIMITY_RANGE_M)
  {
    ROS_INFO("seeing block ...");
    speed = SLOW;
    steps = SHORT;
    return;
  }
  speed = FAST;
  steps = LONG;
  */
}

void MoveBase::rotate(geometry_msgs::Twist &twist)
{
  ros::Rate loop_rate(10);

  while (calc_movement() > 0.8) {
    std::cout << "calc_movement(): " << calc_movement() << "\n";
    twist.linear.x = 0;
    twist.angular.z = calc_movement() * rotateDirection;
    pub_movement.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void MoveBase::move(geometry_msgs::Twist &twist)
{
  twist.angular.z = calc_movement() * rotateDirection;
  twist.linear.x = 0.1 * speed * moveDirection;
  pub_movement.publish(twist);
}

void MoveBase::rotate_to_goal(geometry_msgs::Twist &twist)
{
  ros::Rate loop_rate(10);
  double goal_yaw = curr_goal.yaw;
  double roboter_direction = current_pose.theta;
  if (goal_yaw < 0) { goal_yaw += 6.28; }
  if (roboter_direction < 0) { roboter_direction += 6.28; }
  while(abs(goal_yaw - roboter_direction) > 0.5) {
    twist.linear.x = 0;
    twist.angular.z = 0.5 * (goal_yaw - roboter_direction);
    pub_movement.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
    roboter_direction = current_pose.theta;
    if (roboter_direction < 0) { roboter_direction += 6.28; }
  }
  pubZeroVel(twist);
  // grab ressource
  loop_rate.sleep();
  /**
  while(!grabbed) {
    is_reach_goal = true;
    ros::spinOnce();
  }
  */
}

bool MoveBase::reach_goal(geometry_msgs::Twist &twist)
{
  if (path.size() < 10) {
    rotate_to_goal(twist);
    if (goals.empty() && !request_new_plan) {
      is_job_finished = true;
      stopSim = true;
      return true;
    } else if (!goals.empty()) {
      curr_goal = goals[goals.size() - 1];
      goals.pop_back();
      request_new_plan = true;
    }
    if (Grab::grabbed == false) {
      Grab grab;
      grab.grab_resource();
    }
    sleep(5);
    return true;
  }
  return false;
}

void MoveBase::pubZeroVel(geometry_msgs::Twist &twist)
{
  twist.angular.z = 0;
  twist.linear.x = 0;
  pub_movement.publish(twist);
  ros::spinOnce();
}

void MoveBase::find_nearst_pos() {
  while (calc_distance(path[path.size()-1], roboter_pos)
         > calc_distance(path[path.size()-2], roboter_pos)) {
    path.pop_back();
  }

}

int MoveBase::calc_distance(std::pair<int, int> point1, std::pair<int, int> point2) {
  return signur(point1.first - point2.first) * (point1.first - point2.first) +
         signur(point1.second - point2.second) * (point1.second - point2.second);
}

void MoveBase::calc_coordinate_map(std::pair<int, int> &pixel, CoPair &coordinate) {
  int origin_x = COL / 2;
  int origin_y = ROW / 2;

  double x = ((double) (pixel.second - origin_x)) * mapResolution;
  double y = ((double) (pixel.first - origin_y)) * mapResolution;
  coordinate.first = x;
  coordinate.second = y;
}

void MoveBase::calc_coordinate_matrix(double x, double y, std::pair<int, int> &pixel) {
  int origin_x = COL / 2;
  int origin_y = ROW / 2;

  int i = origin_x + (int) (x / mapResolution);
  int j = origin_y + (int) (y / mapResolution);

  pixel.second = i;
  pixel.first = j;
}

float MoveBase::calc_movement()
{
  int theta_x = path[path.size()-steps].second - roboter_pos.second;
  int theta_y = path[path.size()-steps].first - roboter_pos.first;
  float angle = atan2((double) theta_y, (double) theta_x);
  float theta = current_pose.theta;
  float PI = 3.1416;
  //ROS_ERROR("angle = %f, theta = %f", angle, theta);
  if (angle >= 0 && angle <  PI/ 2) {
    if ( angle - PI/2 < theta && theta <= angle) {
      rotateDirection = ANTICLOCKWISE;
      moveDirection = FORWARD;
      return this->abs(theta - angle);
    } else if (angle < theta && angle + PI/2 >= theta) {
      rotateDirection = CLOCKWISE;
      moveDirection = FORWARD;
      return this->abs(theta - angle);
    } else if (angle - PI < theta && angle - PI/2 >= theta) {
      rotateDirection = CLOCKWISE;
      moveDirection = BACKWARD;
      return this->abs(theta - angle + PI);   //--
    } else {
      rotateDirection = ANTICLOCKWISE;
      moveDirection = BACKWARD;
      if (theta > 0) { return PI - theta + angle; }
      else { return abs(theta - angle + PI); }  //--
    }
  } else if ((angle >= PI / 2) && angle < PI) {
    if (angle - PI/2 < theta && theta <= angle) {
      rotateDirection = ANTICLOCKWISE;
      moveDirection = FORWARD;
      return this->abs(theta - angle);
    } else if (angle - PI < theta && angle - PI/2 >= theta) {
      rotateDirection = CLOCKWISE;
      moveDirection = BACKWARD;
      return this->abs(theta - angle + PI);   //--
    } else if (angle - 3*PI/2 < theta && angle - PI > theta) {
      rotateDirection = ANTICLOCKWISE;
      moveDirection = BACKWARD;
      return this->abs(theta - angle + PI);   //--
    } else {
      rotateDirection = CLOCKWISE;
      moveDirection = FORWARD;
      if (theta < 0) { return 2*PI + theta - angle; }
      else { return theta - angle; }
    }
  } else if (angle >= -(PI / 2) && angle < 0) {
    if (angle < theta && angle + PI/2 >= theta) {
      rotateDirection = CLOCKWISE;
      moveDirection = FORWARD;
      return this->abs(theta - angle);
    } else if (angle - PI/2 < theta && theta < angle) {
      rotateDirection = ANTICLOCKWISE;
      moveDirection = FORWARD;
      return this->abs(theta - angle);
    } else if (angle + PI/2 < theta && angle + PI >= theta) {
      rotateDirection = ANTICLOCKWISE;
      moveDirection = BACKWARD;
      return this->abs(theta - angle - PI);   //--
    } else {
      rotateDirection = CLOCKWISE;
      moveDirection = BACKWARD;
      if (theta < 0) { return PI + theta - angle; }
      else { return theta - angle - PI; }
    }
  } else {
    if (angle < theta && angle + PI/2 >= theta) {
      rotateDirection = CLOCKWISE;
      moveDirection = FORWARD;
      return this->abs(theta - angle);
    } else if (angle + PI/2 < theta && angle + PI >= theta) {
      rotateDirection = ANTICLOCKWISE;
      moveDirection = BACKWARD;
      return this->abs(theta - angle - PI);   //--
    } else if (angle + PI < theta && angle + 3*PI/2 >= theta) {
      rotateDirection = CLOCKWISE;
      moveDirection = BACKWARD;
      return this->abs(theta - angle - PI);
    } else {
      rotateDirection = ANTICLOCKWISE;
      moveDirection = FORWARD;
      if (theta > 0) { return 2*PI - theta + angle; }
      else { return abs(theta - angle);}
    }
  }
}

/**
void MoveBase::pose_Callback(const gazebo_msgs::LinkStates::ConstPtr &msg)
{
    if (is_reach_goal && !grabbed)
    {
        int i = 0;
        while (msg->name[i] != "turtlebot3_burger::gripper_link")
        {
          i++;
          std::cout << "i: " << i << std::endl;
        }

        //for(int j = 0; j < resources_in_simulation; j++)
        //{
            //resources[j].distance = sqrt(pow(msg->pose[i].position.x - resources[j].x, 2) + pow(msg->pose[i].position.y - resources[j].y, 2));
        int j = 1;
            if (!grabbed)
            {
                current_publish_id++;

                gazebo_msgs::ModelState modelstate;
                modelstate.model_name = (std::string) resources[j].name;
                modelstate.pose.position.x = msg->pose[i].position.x;
                modelstate.pose.position.y = msg->pose[i].position.y;
                modelstate.pose.position.z = 0.03;
                modelstate.reference_frame = (std::string) "world";

                ros::Publisher publish_machine = nh.advertise<global_planner_qt::got_resource>("/roboter/resource", 100);
                global_planner_qt::got_resource resource_msg;
                resource_msg.got_r = true;
                resource_msg.resource_name = "WHITE";
                resource_msg.id = current_publish_id;
                resource_msg.object_name = resources[j].name;

                float time = 0;
                while(time < 0.5)
                {
                    publish_machine.publish(resource_msg);
                    publisher.publish(modelstate);
                    ros::Duration(0.05).sleep();
                    time += 0.05;
                }

                current_resource = msg->name[i];
                ROS_ERROR("%s", current_resource.c_str());
                grabbed = true;
            }
        //}
    }
}
*/
/**
void MoveBase::resource_pose_Callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{ 
    if(!found_ressources)
    {
        //Subscriber
        int i = 0;
        int found_obj = 1;
        while(found_obj < 4)
        {
            std::ostringstream convert;
            std::string resource_name = std::string("Resource_White_");

            convert << (found_obj);
            resource_name += convert.str();

            if(msg->name[i] == resource_name)
            {
                resources[found_obj - 1].x = msg->pose[i].position.x;
                resources[found_obj - 1].y = msg->pose[i].position.y;
                resources[found_obj - 1].name = msg->name[i];
                found_obj++;
            }
            i++;
        }
        
        found_ressources = true;
    }
}

void MoveBase::resource_callback(const global_planner_qt::got_resource::ConstPtr& msg)
{
    if(sub_counter != msg->id)
    {
        current_resource = msg->resource_name;
        grabbed = msg->got_r; 
        sub_counter = msg->id;
        resources_in_simulation--;
    }
    
}
*/
} // namespace global_planner


