#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <referee_node/referee_machines.h>

using namespace std;
using namespace ros;

struct Product
{
    string name;
    string *resources;
};


Product products[3];

referee_node::referee_machines machines[3];

referee_node::referee_machines new_Machine(string name, float x, float y) 
{
    referee_node::referee_machines new_machine;
    new_machine.name = name;
    new_machine.x = x;
    new_machine.y = y;
    return new_machine;
}

void publish_Machines(NodeHandle n)
{
    Publisher publisher = n.advertise<referee_node::referee_machines>("referee_machines", 10);
    while (true) {
        publisher.publish(machines[0]);
    }
}

void create_Machines(NodeHandle n) {
    machines[0] = new_Machine("M_red", -1.9 , 2.66);
    machines[1] = new_Machine("M_blue", 2.22 , 2.22);
    machines[2] = new_Machine("M_yellow",1.9, -2.66);
    publish_Machines(n);
}



Product new_Product(string name, string* resources)
{
    Product new_product;
    new_product.name = name;
    new_product.resources = resources;
    return new_product;
}

void publish_Products()
{
    for(int i = 0; i < 3; i++)
    {
        cout << endl << "Name: " << products[i].name << endl;
        for(int j = 0; j < 2; j++)
        {
            cout << "Resource_" << j + 1 << ": " << products[i].resources[j] << endl;
        }
    }
}

void create_Products() 
{
    //Resources
    string orange[2] = {"red", "yellow"};
    string green[2] = {"yellow", "blue"};
    string violet[2] = {"red", "blue"};
    
    //Products
    products[0] = new_Product("orange", orange);
    products[1] = new_Product("green", green);
    products[2] = new_Product("violet", violet);    
    
    //Show products
    publish_Products();
}

void start_Info()
{
    ROS_INFO(" Starting Referee Node ...");
    ROS_INFO("The match will start in 5 sec");
    sleep(1);
    ROS_INFO("4 sec");
    sleep(1);
    ROS_INFO("3 sec");
    sleep(1);
    ROS_INFO("2 sec");
    sleep(1);
    ROS_INFO("1 sec");
    sleep(1);
    ROS_INFO("Go!");
}

void start_KI_Node()
{
    system("xterm -e rosrun roboter_steuerung move_random");
}

int main(int argc, char **argv)
{
    init(argc, argv, "referee_Node");
    NodeHandle n;
    
    //Start program
    //create_Products();
    create_Machines(n);
    return 0;
}
