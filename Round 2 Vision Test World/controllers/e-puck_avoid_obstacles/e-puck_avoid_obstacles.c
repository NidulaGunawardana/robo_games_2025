#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <iostream>

#define TIME_STEP 32  // Webots simulation step
#define MAX_SPEED 6.28  // Maximum motor speed

using namespace webots;
using namespace std;

int main() {
    // Initialize the Webots robot
    Robot *robot = new Robot();

    // Get the left and right motors
    Motor *left_motor = robot->getMotor("left wheel motor");
    Motor *right_motor = robot->getMotor("right wheel motor");

    // Enable speed control
    left_motor->setPosition(INFINITY);
    right_motor->setPosition(INFINITY);
    left_motor->setVelocity(0.0);
    right_motor->setVelocity(0.0);

    // Enable keyboard input
    Keyboard *keyboard = robot->getKeyboard();
    keyboard->enable(TIME_STEP);

    cout << "Use Arrow Keys to Control the Robot (ESC to Exit)" << endl;

    while (robot->step(TIME_STEP) != -1) {
        int key = keyboard->getKey();

        double left_speed = 0.0;
        double right_speed = 0.0;

        // Check which key is pressed
        switch (key) {
            case Keyboard::UP:
                left_speed = MAX_SPEED;
                right_speed = MAX_SPEED;
                cout << "Moving Forward" << endl;
                break;
            case Keyboard::DOWN:
                left_speed = -MAX_SPEED;
                right_speed = -MAX_SPEED;
                cout << "Moving Backward" << endl;
                break;
            case Keyboard::LEFT:
                left_speed = -MAX_SPEED / 2;
                right_speed = MAX_SPEED / 2;
                cout << "Turning Left" << endl;
                break;
            case Keyboard::RIGHT:
                left_speed = MAX_SPEED / 2;
                right_speed = -MAX_SPEED / 2;
                cout << "Turning Right" << endl;
                break;
            case Keyboard::END:  // ESC key
                cout << "Exiting..." << endl;
                delete robot;
                return 0;
            default:
                cout << "Stopped" << endl;
                break;
        }

        // Set motor speeds
        left_motor->setVelocity(left_speed);
        right_motor->setVelocity(right_speed);
    }

    delete robot;
    return 0;
}
