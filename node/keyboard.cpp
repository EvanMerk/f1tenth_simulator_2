#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/String.hpp>

#include <termios.h>

#include <stdio.h>
#include <signal.h>

// for printing
#include <iostream>

static volatile sig_atomic_t keep_running = 1;


void sigHandler(int not_used) {
    keep_running = 0;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv, "keyboard");
    // Initialize Node Handle
    auto node = rclcpp::Node::make_shared("keyboard")

    // Initialze publisher
    std::string keyboard_topic;
    node->get_parameter("keyboard_topic", keyboard_topic);

    rclcpp::Publisher key_pub = node->advertise<std_msgs::msg::String>(keyboard_topic, 10);


    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr( STDIN_FILENO, 0, &newt);

    struct sigaction act;
    act.sa_handler = sigHandler;
    sigaction(SIGINT, &act, NULL);
    

    std_msgs::msg::String msg;
    int c;
    while ((rclcpp::ok()) && (keep_running)) {
        // get the character pressed
        c = getchar();

        // Publish the character 
        msg.data = c;
        key_pub.publish(msg);
    }

    tcsetattr( STDIN_FILENO, 0, &oldt);
    
    return 0;
}