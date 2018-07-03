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

//extern bool stopSim;

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

extern bool update_status;

#endif // MUTEX_H
