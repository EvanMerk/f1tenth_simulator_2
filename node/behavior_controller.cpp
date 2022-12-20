#include <rclcpp/rclcpp.hpp>
#include <rclcpp/ament_package.hpp>

#include <std_msgs/msg/Int32MultiArray.hpp>
#include <std_msgs/msg/Bool.hpp>
#include <sensor_msgs/msg/Joy.h>
#include <sensor_msgs/msg/LaserScanode->h>
#include <nav_msgs/msg/Odometry.h>
#include <sensor_msgs/msg/Imu.h>
#include <std_msgs/msg/String.hpp>

#include <fstream>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/precompute.hpp"

using namespace racecar_simulator;

class BehaviorController {
private:
    // A ROS node
    rclcpp::NodeHandle n;

    // Listen for messages from joystick, keyboard, laser scan, odometry, and IMU
    rclcpp::Subscriber joy_sub;
    rclcpp::Subscriber key_sub;
    rclcpp::Subscriber laser_sub;
    rclcpp::Subscriber odom_sub;
    rclcpp::Subscriber imu_sub;
    rclcpp::Subscriber brake_bool_sub;

    // Publisher for mux controller
    rclcpp::Publisher mux_pub;

    // Mux indices
    int joy_mux_idx;
    int key_mux_idx;
    int random_walker_mux_idx;
    int nav_mux_idx;
    int brake_mux_idx;
    // ***Add mux index for new planner here***
    // int new_mux_idx;

    // Mux controller array
    std::vector<bool> mux_controller;
    int mux_size;

    // Button indices
    int joy_button_idx;
    int key_button_idx;
    int random_walk_button_idx;
    int brake_button_idx;
    int nav_button_idx;
    // ***Add button index for new planner here***
    // int new_button_idx;

    // Key indices
    std::string joy_key_char;
    std::string keyboard_key_char;
    std::string brake_key_char;
    std::string random_walk_key_char;
    std::string nav_key_char;
    // ***Add key char for new planner here***
    // int new_key_char;

    // Is ebrake on? (not engaged, but on)
    bool safety_on;

    // To roughly keep track of vehicle state
    racecar_simulator::CarState state;

    // precompute distance from lidar to edge of car for each beam
    std::vector<double> car_distances;

    // precompute cosines of scan angles
    std::vector<double> cosines;

    // for collision detection
    double ttc_threshold;
    bool in_collision=false;

    // for collision logging
    std::ofstream collision_file;
    double beginning_seconds;
    int collision_count=0;


public:
    BehaviorController() {
        // Initialize the node handle
        auto node = rclcpp::Node::make_shared("~")

        // get topic names
        std::string scan_topic, odom_topic, imu_topic, joy_topic, keyboard_topic, brake_bool_topic, mux_topic;
        node->get_parameter("scan_topic", scan_topic);
        node->get_parameter("odom_topic", odom_topic);
        node->get_parameter("imu_topic", imu_topic);
        node->get_parameter("joy_topic", joy_topic);
        node->get_parameter("mux_topic", mux_topic);
        node->get_parameter("keyboard_topic", keyboard_topic);
        node->get_parameter("brake_bool_topic", brake_bool_topic);

        // Make a publisher for mux messages
        mux_pub = node->advertise<std_msgs::msg::Int32MultiArray>(mux_topic, 10);

        // Start subscribers to listen to laser scan, joy, IMU, and odom messages
        laser_sub = node->subscribe(scan_topic, 1, &BehaviorController::laser_callback, this);
        joy_sub = node->subscribe(joy_topic, 1, &BehaviorController::joy_callback, this);
        imu_sub = node->subscribe(imu_topic, 1, &BehaviorController::imu_callback, this);
        odom_sub = node->subscribe(odom_topic, 1, &BehaviorController::odom_callback, this);
        key_sub = node->subscribe(keyboard_topic, 1, &BehaviorController::key_callback, this);
        brake_bool_sub = node->subscribe(brake_bool_topic, 1, &BehaviorController::brake_callback, this);

        // Get mux indices
        node->get_parameter("joy_mux_idx", joy_mux_idx);
        node->get_parameter("key_mux_idx", key_mux_idx);
        node->get_parameter("random_walker_mux_idx", random_walker_mux_idx);
        node->get_parameter("brake_mux_idx", brake_mux_idx);
        node->get_parameter("nav_mux_idx", nav_mux_idx);
        // ***Add mux index for new planner here***
        // node->get_parameter("new_mux_idx", new_mux_idx);

        // Get button indices
        node->get_parameter("joy_button_idx", joy_button_idx);
        node->get_parameter("key_button_idx", key_button_idx);
        node->get_parameter("random_walk_button_idx", random_walk_button_idx);
        node->get_parameter("brake_button_idx", brake_button_idx);
        node->get_parameter("nav_button_idx", nav_button_idx);
        // ***Add button index for new planner here***
        // node->get_parameter("new_button_idx", new_button_idx);

        // Get key indices
        node->get_parameter("joy_key_char", joy_key_char);
        node->get_parameter("keyboard_key_char", keyboard_key_char);
        node->get_parameter("random_walk_key_char", random_walk_key_char);
        node->get_parameter("brake_key_char", brake_key_char);
        node->get_parameter("nav_key_char", nav_key_char);
        // ***Add key char for new planner here***
        // node->get_parameter("new_key_char", new_key_char);

        // Initialize the mux controller 
        node->get_parameter("mux_size", mux_size);
        mux_controller.reserve(mux_size);
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }

        // Start with ebrake off
        safety_on = false;

        // Initialize state
        state = {.x=0.0, .y=0.0, .theta=0.0, .velocity=0.0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};

        // Get params for precomputation and collision detection
        int scan_beams;
        double scan_fov, scan_ang_incr, wheelbase, width, scan_distance_to_base_link;
        node->get_parameter("ttc_threshold", ttc_threshold);
        node->get_parameter("scan_beams", scan_beams);
        node->get_parameter("scan_distance_to_base_link", scan_distance_to_base_link);
        node->get_parameter("width", width);
        node->get_parameter("wheelbase", wheelbase);
        node->get_parameter("scan_field_of_view", scan_fov);
        scan_ang_incr = scan_fov / scan_beams;

        // Precompute cosine and distance to car at each angle of the laser scan
        cosines = Precompute::get_cosines(scan_beams, -scan_fov/2.0, scan_ang_incr);
        car_distances = Precompute::get_car_distances(scan_beams, wheelbase, width, 
                scan_distance_to_base_link, -scan_fov/2.0, scan_ang_incr);

        // Create collision file to be written to
        std::string filename;
        node->get_parameter("collision_file", filename);
        collision_file.open(rclcpp::package::getPath("f1tenth_simulator") + "/logs/" + filename + ".txt");
        beginning_seconds = rclcpp::Time::now().toSec();

    }

    /// ---------------------- GENERAL HELPER FUNCTIONS ----------------------

    void publish_mux() {
        // make mux message
        std_msgs::msg::Int32MultiArray mux_msg;
        mux_msg.data.clear();
        // push data onto message
        for (int i = 0; i < mux_size; i++) {
            mux_msg.data.push_back(int(mux_controller[i]));
        }

        // publish mux message
        mux_pub.publish(mux_msg);
    }

    void change_controller(int controller_idx) {
        // This changes the controller to the input index and publishes it

        // turn everything off
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }
        // turn on desired controller
        mux_controller[controller_idx] = true;

        publish_mux();
    }

    void collision_checker(const sensor_msgs::msg::LaserScan & msg) {
        // This function calculates TTC to see if there's a collision
        if (state.velocity != 0) {
            for (size_t i = 0; i < msg.ranges.size(); i++) {
                double angle = msg.angle_min + i * msg.angle_increment;

                // calculate projected velocity
                double proj_velocity = state.velocity * cosines[i];
                double ttc = (msg.ranges[i] - car_distances[i]) / proj_velocity;

                // if it's small, there's a collision
                if ((ttc < ttc_threshold) && (ttc >= 0.0)) { 
                    // Send a blank mux and write to file
                    collision_helper();

                    in_collision = true;

                    collision_count++;
                    collision_file << "Collision #" << collision_count << " detected:\n";
                    collision_file << "TTC: " << ttc << " seconds\n";
                    collision_file << "Angle to obstacle: " << angle << " radians\n";
                    collision_file << "Time since start of sim: " << (rclcpp::Time::now().toSec() - beginning_seconds) << " seconds\n";
                    collision_file << "\n";
                    return;
                }
            }
            // if it's gone through all beams without detecting a collision, reset in_collision
            in_collision = false;
        }
    }

    void collision_helper() {
        // This function will turn off ebrake, clear the mux and publish it

        safety_on = false;

        // turn everything off
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }

        publish_mux();
    }

    void toggle_mux(int mux_idx, std::string driver_name) {
        // This takes in an index and the name of the planner/driver and 
        // toggles the mux appropiately
        if (mux_controller[mux_idx]) {
            ROS_INFO_STREAM(driver_name << " turned off");
            mux_controller[mux_idx] = false;
            publish_mux();
        }
        else {
            ROS_INFO_STREAM(driver_name << " turned on");
            change_controller(mux_idx);
        }
    }

    void toggle_brake_mux() {
        ROS_INFO_STREAM("Emergency brake engaged");
        // turn everything off
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }
        // turn on desired controller
        mux_controller[brake_mux_idx] = true;

        publish_mux();
    }


    /// ---------------------- CALLBACK FUNCTIONS ----------------------

    void brake_callback(const std_msgs::msg::Bool & msg) {
        if (msg.data && safety_on) {
            toggle_brake_mux();
        } else if (!msg.data && mux_controller[brake_mux_idx]) {
            mux_controller[brake_mux_idx] = false;
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy & msg) {
        // Changing mux_controller:
        if (msg.buttons[joy_button_idx]) { 
            // joystick
            toggle_mux(joy_mux_idx, "Joystick");
        }
        if (msg.buttons[key_button_idx]) { 
            // keyboard
            toggle_mux(key_mux_idx, "Keyboard");
        }
        else if (msg.buttons[brake_button_idx]) { 
            // emergency brake 
            if (safety_on) {
                ROS_INFO("Emergency brake turned off");
                safety_on = false;
            }
            else {
                ROS_INFO("Emergency brake turned on");
                safety_on = true;
            }
        }
        else if (msg.buttons[random_walk_button_idx]) { 
            // random walker
            toggle_mux(random_walker_mux_idx, "Random Walker");
        } else if (msg.buttons[nav_button_idx]) {
            // nav
            toggle_mux(nav_mux_idx, "Navigation");
        }
        // ***Add new else if statement here for new planning method***
        // if (msg.buttons[new_button_idx]) {
        //  // new planner
        //  toggle_mux(new_mux_idx, "New Planner");
        // }

    }

    void key_callback(const std_msgs::msg::String & msg) {
        // Changing mux controller:
        if (msg.data == joy_key_char) {
            // joystick
            toggle_mux(joy_mux_idx, "Joystick");
        } else if (msg.data == keyboard_key_char) {
            // keyboard
            toggle_mux(key_mux_idx, "Keyboard");
        } else if (msg.data == brake_key_char) {
            // emergency brake 
            if (safety_on) {
                ROS_INFO("Emergency brake turned off");
                safety_on = false;
            }
            else {
                ROS_INFO("Emergency brake turned on");
                safety_on = true;
            }
        } else if (msg.data == random_walk_key_char) {
            // random walker
            toggle_mux(random_walker_mux_idx, "Random Walker");
        } else if (msg.data == nav_key_char) {
            // nav
            toggle_mux(nav_mux_idx, "Navigation");
        }
        // ***Add new else if statement here for new planning method***
        // if (msg.data == new_key_char) {
        //  // new planner
        //  toggle_mux(new_mux_idx, "New Planner");
        // }

    }

    void laser_callback(const sensor_msgs::msg::LaserScan & msg) {
        // check for a collision
        collision_checker(msg);


    }

    void odom_callback(const nav_msgs::msg::Odometry & msg) {
        // Keep track of state to be used elsewhere
        state.velocity = msg.twist.twist.linear.x;
        state.angular_velocity = msg.twist.twist.angular.z;
        state.x = msg.pose.pose.positionode->x;
        state.y = msg.pose.pose.positionode->y;
    }

    void imu_callback(const sensor_msgs::msg::Imu & msg) {

    }


};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv, "behavior_controller");
    BehaviorController bc;
    rclcpp::spin();
    return 0;
}
