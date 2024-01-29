# drone_controller
simple package to bebop2 control 



sudo apt update
sudo apt upgrade

sudo add-apt-repository 'deb http://archive.ubuntu.com/ubuntu bionic main'

sudo apt update
sudo apt install g++-7
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 60
g++ --version



#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "bebop_takeoff_land");
    ros::NodeHandle nh;

    // Publisher para enviar comandos de despegue y aterrizaje
    ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
    ros::Publisher land_pub = nh.advertise<std_msgs::Empty>("/bebop/land", 1);

    // Esperar un breve momento para que los nodos estén listos
    sleep(1);

    // Despegar
    ROS_INFO("Despegando...");
    std_msgs::Empty takeoff_msg;
    takeoff_pub.publish(takeoff_msg);
    sleep(5); // Esperar 5 segundos para el despegue

    // Aterrizar
    ROS_INFO("Aterrizando...");
    std_msgs::Empty land_msg;
    land_pub.publish(land_msg);
    sleep(3); // Esperar 3 segundos para el aterrizaje

    // Cerrar el nodo ROS
    ros::shutdown();

    return 0;
}
