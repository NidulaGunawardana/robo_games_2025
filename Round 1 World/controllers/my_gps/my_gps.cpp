#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Gyro.hpp>
#include <iostream>
#include <cmath>

using namespace webots;
using namespace std;

// Define a custom robot class inheriting from the Webots Robot class
class MyRobot : public Robot {
public:
    MyRobot() {
        // Get the simulation time step
        timeStep = static_cast<int>(getBasicTimeStep());

        // Initialize robot devices
        gps = getGPS("gps");
        leftMotor = getMotor("left wheel motor");
        rightMotor = getMotor("right wheel motor");
        ds_front = getDistanceSensor("sharp_front");
        ds_front_left = getDistanceSensor("sharp_front_left");
        ds_front_right = getDistanceSensor("sharp_front_right");
        ds_left = getDistanceSensor("sharp_left");
        ds_right = getDistanceSensor("sharp_right");
        left_wheel_sensor = getPositionSensor("left wheel sensor");
        right_wheel_sensor = getPositionSensor("right wheel sensor");
        gyro = getGyro("gyro");

        // Enable sensors
        gps->enable(timeStep);
        ds_front->enable(timeStep);
        ds_front_left->enable(timeStep);
        ds_front_right->enable(timeStep);
        ds_left->enable(timeStep);
        ds_right->enable(timeStep);
        left_wheel_sensor->enable(timeStep);
        right_wheel_sensor->enable(timeStep);
        gyro->enable(timeStep);

        // Set motors to velocity control mode with initial velocity set to 0
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    // Method to read distance sensors and determine the robot's surroundings
    int getDistanceSensors() {
        double distanceFront = getDistance(ds_front);
        double distanceLeft = getDistance(ds_left);
        double distanceRight = getDistance(ds_right);

        cout << "Distance sensor values: Front=" << distanceFront << ", Left=" << distanceLeft << ", Right=" << distanceRight << endl;

        // Evaluate sensor readings and return corresponding disjunction identifier
        if (distanceFront > 0.39 && distanceLeft < 0.5 && distanceRight < 0.5)
            return 1;
        else if (distanceFront < 0.39 && distanceLeft > 0.5 && distanceRight < 0.5)
            return 2;
        else if (distanceFront < 0.39 && distanceLeft < 0.5 && distanceRight > 0.5)
            return 3;
        else if (distanceFront > 0.39 && distanceLeft > 0.5 && distanceRight < 0.5)
            return 4;
        else if (distanceFront > 0.39 && distanceLeft < 0.5 && distanceRight > 0.5)
            return 5;
        else if (distanceFront < 0.39 && distanceLeft > 0.5 && distanceRight > 0.5)
            return 6;
        else if (distanceFront > 0.39 && distanceLeft > 0.5 && distanceRight > 0.5)
            return 7;
        else
            return 0; // No significant obstacles detected
    }

    // Move the robot forward by a specified distance (in simulation steps)
    void goForward(int steps) {
        cout << "Going forward for " << steps << " steps..." << endl;

        // PID controller parameters
        double Kp = 0.03;
        double Ki = 0.0;
        double Kd = 0.01;
        double previousError = 0.0;
        double integral = 0.0;

        for (int i = 0; i < steps; i++) {
            double initialLeftPosition = getLeftWheelSensor();

            // Move forward until the left wheel rotates a specific distance
            while ((getLeftWheelSensor() - initialLeftPosition) < 12) {
                // Update global distance sensor readings midway
                if ((getLeftWheelSensor() - initialLeftPosition) >= 5.95 && (getLeftWheelSensor() - initialLeftPosition) < 6.1) {
                    globalDistance = getDistanceSensors();
                }

                // Wall following using PID control
                double distanceLeft = getDistance(ds_left);
                double distanceRight = getDistance(ds_right);
                double error = distanceLeft - distanceRight;
                integral += error;
                double derivative = error - previousError;
                double correction = Kp * error + Ki * integral + Kd * derivative;
                previousError = error;

                // Adjust motor velocities based on PID correction
                double leftSpeed = 4 + correction;
                double rightSpeed = 4 - correction;

                leftMotor->setVelocity(leftSpeed);
                rightMotor->setVelocity(rightSpeed);
                step(timeStep); // Advance simulation
            }

            stopRobot(); // Stop robot after moving the required distance
        }
    }

    // Turn the robot left by 90 degrees using wheel encoders
    void turnLeft() {
        double initialLeftWheel = getLeftWheelSensor();
        double targetLeftWheel = initialLeftWheel - 2.241; // Approx. rotation for 90 degrees

        // Rotate until the left wheel reaches the target position
        while (getLeftWheelSensor() > targetLeftWheel) {
            leftMotor->setVelocity(-0.8);
            rightMotor->setVelocity(0.8);
            step(timeStep);
        }
        stopRobot();
    }

    // Turn the robot right by 90 degrees using wheel encoders
    void turnRight() {
        double initialRightWheel = getRightWheelSensor();
        double targetRightWheel = initialRightWheel - 2.241; // Approx. rotation for 90 degrees

        // Rotate until the right wheel reaches the target position
        while (getRightWheelSensor() > targetRightWheel) {
            leftMotor->setVelocity(0.8);
            rightMotor->setVelocity(-0.8);
            step(timeStep);
        }
        stopRobot();
    }

    // Align the robot with the wall using the front distance sensor
    void align_wall() {
        double distanceFront = getDistance(ds_front);

        while (distanceFront < 0.89 || distanceFront > 0.9) {
            if (distanceFront < 0.89) {
                leftMotor->setVelocity(0.5);
                rightMotor->setVelocity(0.5);
            } else if (distanceFront > 0.9) {
                leftMotor->setVelocity(-0.5);
                rightMotor->setVelocity(-0.5);
            }
            step(timeStep);
            distanceFront = getDistance(ds_front);
        }
        stopRobot();
    }

    // Make the robot parallel to the wall using front-left and front-right distance sensors
    void parallel_wall() {
        double distanceLeft = getDistance(ds_front_left);
        double distanceRight = getDistance(ds_front_right);
        double distanceDifference = distanceLeft - distanceRight;

        while (fabs(distanceDifference) > 0.001) { // Continue until the difference is close to zero
            if (distanceDifference > 0.01) {
                leftMotor->setVelocity(-0.05);
                rightMotor->setVelocity(0.05);
            } else if (distanceDifference < -0.01) {
                leftMotor->setVelocity(0.05);
                rightMotor->setVelocity(-0.05);
            }
            step(timeStep);
            distanceLeft = getDistance(ds_front_left);
            distanceRight = getDistance(ds_front_right);
            distanceDifference = distanceLeft - distanceRight;
        }
        stopRobot();
    }

    // Stop the robot by setting motor velocities to zero
    void stopRobot() {
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    // Get distance reading from a specific distance sensor
    double getDistance(DistanceSensor *ds) {
        return ds->getValue();
    }

    // Get the current reading of the left wheel position sensor
    double getLeftWheelSensor() {
        return left_wheel_sensor->getValue();
    }

    // Get the current reading of the right wheel position sensor
    double getRightWheelSensor() {
        return right_wheel_sensor->getValue();
    }

    // Structure to represent a grid cell
    struct Cell {
        int row;
        int column;
    };

    // Calculate the current grid cell based on GPS coordinates
    Cell calculateCell(double x, double y) {
        const double leftX = 1.25;
        const double topY = -1.25;
        const double cellWidth = 0.25;
        const double cellHeight = 0.25;
        const int columns = 9;

        int column = static_cast<int>((leftX - x) / cellWidth);
        int row = static_cast<int>((y - topY) / cellHeight);

        if (column < 0 || column > columns || row < 0 || row > columns) {
            return {-1, -1}; // Out of bounds
        }

        column = columns - column; // Reverse column order
        return {row, column};
    }

    // Main simulation loop
    void run() {
        cout << "Starting simulation loop..." << endl;

        // Initial delay (3 seconds)
        for (int i = 0; i < 1000 / timeStep; ++i) {
            step(timeStep);
        }

        // Get initial distance sensor readings
        globalDistance = getDistanceSensors();

        while (step(timeStep) != -1) {
            // Perform actions based on sensor readings
            switch (globalDistance) {
            case 1:
                turnLeft();
                goForward(1);
                break;
            case 2:
                goForward(1);
                break;
            case 3:
                turnLeft();
                goForward(1);
                break;
            case 4:
                align_wall();
                parallel_wall();
                turnRight();
                goForward(1);
                break;
            case 5:
                align_wall();
                parallel_wall();
                turnLeft();
                goForward(1);
                break;
            case 6:
                goForward(1);
                break;
            case 7:
                align_wall();
                parallel_wall();
                turnRight();
                stopRobot();
                align_wall();
                parallel_wall();
                turnRight();
                goForward(1);
                break;
            default:
                break;
            }

            // Print front distance sensor value
            double distanceFront = getDistance(ds_front);
            cout << "Distance sensor values: Front=" << distanceFront << endl;

            // Log robot's position and cell
            const double *gpsValues = gps->getValues();
            double x = gpsValues[0];
            double y = gpsValues[1];

            Cell cell = calculateCell(x, y);
            int row = cell.row;
            int column = cell.column;

            if (row != -1 && column != -1) {
                cout << "GPS Coordinates: X=" << x << ", Y=" << y << " | Row =" << row << ", Column =" << column << endl;
            }
        }
    }

private:
    int timeStep;
    int globalDistance;
    GPS *gps;
    Motor *leftMotor;
    Motor *rightMotor;
    DistanceSensor *ds_front;
    DistanceSensor *ds_front_left;
    DistanceSensor *ds_front_right;
    DistanceSensor *ds_left;
    DistanceSensor *ds_right;
    PositionSensor *left_wheel_sensor;
    PositionSensor *right_wheel_sensor;
    Gyro *gyro;
};

// Entry point of the program
int main() {
    MyRobot robot; // Create an instance of the robot
    robot.run();   // Start the simulation loop
    return 0;
}
