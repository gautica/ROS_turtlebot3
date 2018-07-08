#ifndef MUTEX_H
#define MUTEX_H
#include <vector>

enum Product
{
  NO_PRODUCT = -1,
  BLUE_PRODUCT = 0,
  YELLOW_PRODUCT = 1,
  RED_PRODUCT = 2,
  VIOLET_PRODUCT = 3,
  ORANGE_PRODUCT = 4,
  GREEN_PRODUCT = 5,
  BLACK_PRODUCT = 6
};

enum GameMode
{
  KI_VS_KI = 0,
  PLAYER_VS_PLAYER = 1,
  AS_TEAM = 2
};

enum Robot_ID
{
  ROBOT_0 = 0,
  ROBOT_1 = 1
};

//extern bool stopSim;
extern std::vector<std::vector<int> > gridMap;
extern std::vector<std::pair<int, int> > path_robot0;
extern std::vector<std::pair<int, int> > path_robot1;
extern std::vector<std::vector<int> > roboter_local_field;
extern std::pair<int, int> robot0_pos;
extern std::pair<int, int> robot1_pos;

extern int curr_product_robo0;
extern int curr_product_robo1;

extern std::vector<int> products;
extern std::vector<int> products_copy;

extern unsigned char* robot0_buffer;
extern unsigned int robot0_width;
extern unsigned int robot0_height;

extern unsigned char* robot0_camera_up_buffer;
extern unsigned int robot0_camera_up_width;
extern unsigned int robot0_camera_up_height;

extern unsigned char* robot1_buffer;
extern unsigned int robot1_width;
extern unsigned int robot1_height;

extern unsigned char* robot1_camera_up_buffer;
extern unsigned int robot1_camera_up_width;
extern unsigned int robot1_camera_up_height;
//extern unsigned char* arena_buffer;
//extern unsigned int arena_width;
//extern unsigned int arena_height;

extern bool init_image_robot0;
extern bool init_image_robot1;

extern bool init_camera_up_robot0;
extern bool init_camera_up_robot1;
//extern bool init_image_arena;

extern int curr_gamemode;
extern int ROW;
extern int COL;
extern double mapResolution;
extern bool update_status;
extern bool is_map_init;
extern bool is_robot0_pos_init;
extern bool is_robot1_pos_init;
extern bool is_robot0_path_init;
extern bool is_robot1_path_init;
extern bool is_mapViewr_active;

// for play vs player mode
extern bool go;
extern bool player1_topview;
extern bool player2_topview;

extern bool is_finished_robot0;
extern bool is_finished_robot1;
extern int who_wins;

#endif // MUTEX_H
