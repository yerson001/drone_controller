// file to replicate with opencv rviz and ros
#include "MarDrone/drone_controller.h"
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
namespace MarDrone {

drone_controller::drone_controller(ros::NodeHandle& nh) :
  speed(0.3),
  turn(0.6),
  //cap(0),
  //button1(50, 50, 100, 50, "Botón 1", cv::Scalar(0, 255, 0)),
  //button2(200, 50, 100, 50, "Botón 2", cv::Scalar(0, 0, 255)),
  //button3(350, 50, 100, 50, "Cerrar", cv::Scalar(255, 0, 0)),
  brightnessValue(0)
{
    pub_ = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);
    pub_takeoff = nh.advertise<std_msgs::Empty>("bebop/takeoff", 1);
    pub_land = nh.advertise<std_msgs::Empty>("bebop/land", 1);
    pub_camera = nh.advertise<geometry_msgs::Twist>("bebop/camera_control", 1);
    image_subscriber = nh.subscribe("bebop2/camera_base/image_raw", 1, &drone_controller::imageCallback, this);

    moveBindings = {
      {'i', {1, 0, 0, 0}},
      {'o', {1, 0, 0, -1}},
      {'j', {0, 0, 0, 1}},
      {'l', {0, 0, 0, -1}},
      {'u', {1, 0, 0, 1}},
      {',', {-1, 0, 0, 0}},
      {'.', {-1, 0, 0, 1}},
      {'m', {-1, 0, 0, -1}},
      {'O', {1, -1, 0, 0}},
      {'I', {1, 0, 0, 0}},
      {'J', {0, 1, 0, 0}},
      {'L', {0, -1, 0, 0}},
      {'U', {1, 1, 0, 0}},
      {'<', {-1, 0, 0, 0}},
      {'>', {-1, -1, 0, 0}},
      {'M', {-1, 1, 0, 0}},
      {'t', {0, 0, 1, 0}},
      {'b', {0, 0, -1, 0}},
    };

    speedBindings = {
      {'q', {1.1, 1.1}},
      {'z', {.9, .9}},
      {'w', {1.1, 1}},
      {'x', {.9, 1}},
      {'e', {1, 1.1}},
      {'c', {1, .9}},
    };
    msg = R"(
    Reading from the keyboard and Publishing to Twist!
    ---------------------------
    Moving around:
       u    i    o
       j    k    l
       m    ,    .

    For Holonomic mode (strafing), hold down the shift key:
    ---------------------------
       U    I    O
       J    K    L
       M    <    >

    't : up (+z)
    b : down (-z)

    anything else : stop

    q/z : increase/decrease max speeds by 10%
    w/x : increase/decrease only linear speed by 10%
    e/c : increase/decrease only angular speed by 10%

    ----------------------------------------------------------
    TakeOff            - Press 1
    Landing            - Press 2
    MoveCameraDown     - Press 3
    MoveCameraForward  - Press 4
    ----------------------------------------------------------
    CTRL-C to quit
    )";
/*
    if (!cap.isOpened()) {
        std::cerr << "Error al abrir la cámara" << std::endl;
        exit(EXIT_FAILURE);
    }
*/
    //cv::namedWindow("Interfaz", cv::WINDOW_NORMAL);
    //cv::resizeWindow("Interfaz", 800, 600); // Tamaño fijo de la ventana

    //cv::createTrackbar("Brillo", "Interfaz", &brightnessValue, 100, nullptr);
    //cv::setMouseCallback("Interfaz", onMouse, this);

}
void drone_controller::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Convertir el mensaje de imagen a una matriz de OpenCV
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // Mostrar la imagen en una ventana de OpenCV
        cv::imshow("Drone Image", frame);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Error al convertir la imagen ROS a OpenCV: %s", e.what());
    }
}

void drone_controller::moveCameraDown() {
    geometry_msgs::Twist cam_twist;
    cam_twist.angular.x = 0;
    cam_twist.angular.y = -84;
    cam_twist.angular.z = 0;
    pub_camera.publish(cam_twist);
    std::cout << "angle_camera: " << cam_twist.angular.y << std::endl;
}

void drone_controller::moveCameraForward() {
    geometry_msgs::Twist cam_twist;
    cam_twist.angular.x = 0;
    cam_twist.angular.y = 0;
    cam_twist.angular.z = 0;
    pub_camera.publish(cam_twist);
    std::cout << "angle_camera: " << cam_twist.angular.y << std::endl;
}

int drone_controller::getKey() {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

void drone_controller::printMsg() {
  std::cout << msg << std::endl;
  std::cout << "currently:\tspeed " << speed << "\tturn " << turn << std::endl;
}

void drone_controller::controlBebop(char key){
  double x = 0;
  double y = 0;
  double z = 0;
  double th = 0;

  if (moveBindings.find(key) != moveBindings.end())
  {
      x = std::get<0>(moveBindings[key]);
      y = std::get<1>(moveBindings[key]);
      z = std::get<2>(moveBindings[key]);
      th = std::get<3>(moveBindings[key]);
  }
  else if (speedBindings.find(key) != speedBindings.end())
  {
      speed = speed * std::get<0>(speedBindings[key]);
      turn = turn * std::get<1>(speedBindings[key]);

      std::cout << "currently:\tspeed " << speed << "\tturn " << turn << std::endl;
  }
  else if (key == '1')
  {
      std_msgs::Empty empty_msg;
      std::cout << "key 1 pressed" << std::endl;
      pub_takeoff.publish(empty_msg);
  }
  else if (key == '2')
  {
      std_msgs::Empty empty_msg;
      std::cout << "key 2 pressed" << std::endl;
      pub_land.publish(empty_msg);
  }
  else if (key == '3')
  {
      std::cout << "key 3 pressed" << std::endl;
      moveCameraDown();
  }
  else if (key == '4')
  {
      std::cout << "key 4 pressed" << std::endl;
      moveCameraForward();
  }
  else
  {
      x = 0;
      y = 0;
      z = 0;
      th = 0;
      if (key == 3) // Ctrl-C
      {
          ros::shutdown();
          exit(0);
      }
  }

  geometry_msgs::Twist twist;
  twist.linear.x = x * speed;
  twist.linear.y = y * speed;
  twist.linear.z = z * speed;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = th * turn;
  pub_.publish(twist);
}


void drone_controller::run() {
    ros::Rate loop_rate(10);
    char key;
    printMsg();
    while (ros::ok()) {
        key = getKey();
        std::cout << "key: " << key << std::endl;
        controlBebop(key);
        //ros::spinOnce();
        loop_rate.sleep();
    }
}

} // namespace MarDrone
