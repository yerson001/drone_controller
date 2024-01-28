#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
namespace MarDrone {

class drone_controller
{
private:
    ros::Publisher pub_;
    ros::Publisher pub_takeoff;
    ros::Publisher pub_land;
    ros::Publisher pub_camera;
    ros::Subscriber image_subscriber; // Cambio: es un Subscriber
    ros::NodeHandle nh; // Nuevo: nodo ROS
    double speed;
    double turn;
    std::string msg;
    // gui_button items
    cv::VideoCapture cap;
    cv::Mat frame;

    int brightnessValue;

    std::map<char, std::tuple<double, double, double, double>> moveBindings;
    std::map<char, std::tuple<double, double>> speedBindings;

    void moveCameraDown();
    void moveCameraForward();
    int getKey();
    void controlBebop(char key);
    void printMsg();
    //static void onMouse(int event, int x, int y, int flags, void* userdata);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg); // Cambio: declaración de la función imageCallback
public:
    drone_controller(ros::NodeHandle& nh);
    void run();
};

} // namespace MarDrone

#endif // DRONE_CONTROLLER_H
