#include <ros/ros.h>
#include "MarDrone/drone_controller.h" // Incluye el archivo de encabezado de la clase drone_controller

int main(int argc, char **argv) {
    ros::init(argc, argv, "app_node");
    ros::NodeHandle nh;

    // Crea una instancia de la clase drone_controller
    MarDrone::drone_controller miControlador(nh); // Pasa nh al constructor si es necesario

    // Ejecuta la función run() de la clase drone_controller
    miControlador.run();

    // Resto del código de tu nodo...
    return 0;
}
