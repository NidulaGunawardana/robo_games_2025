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

class MyRobot : public Robot
{
public:
    MyRobot()
    {
        // Get the time step of the simulation
        timeStep = static_cast<int>(getBasicTimeStep());

        // Initialize devices
        gps = getGPS("gps");
        leftMotor = getMotor("left wheel motor");
        rightMotor = getMotor("right wheel motor");
        ds_front = getDistanceSensor("sharp_front");
        ds_left = getDistanceSensor("sharp_left");
        ds_right = getDistanceSensor("sharp_right");
        left_wheel_sensor = getPositionSensor("left wheel sensor");
        right_wheel_sensor = getPositionSensor("right wheel sensor");
        gyro = getGyro("gyro");

        // Enable devices
        gps->enable(timeStep);
        ds_front->enable(timeStep);
        ds_left->enable(timeStep);
        ds_right->enable(timeStep);
        left_wheel_sensor->enable(timeStep);
        right_wheel_sensor->enable(timeStep);
        gyro->enable(timeStep);

        // Set up the motor speeds at 50% of the MAX_SPEED.
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    int getDistanceSensors()
    {
        double distanceFront = getDistance(ds_front);
        double distanceLeft = getDistance(ds_left);
        double distanceRight = getDistance(ds_right);

        cout << "Distance sensor: " << distanceFront << " " << distanceLeft << " " << distanceRight << endl;

        if ((distanceFront > 0.39) & (distanceLeft < 0.5) & (distanceRight < 0.5))
        {
            return 1;
        }
        else if ((distanceFront < 0.39) & (distanceLeft > 0.5) & (distanceRight < 0.5))
        {
            return 2;
        }
        else if ((distanceFront < 0.39) & (distanceLeft < 0.5) & (distanceRight > 0.5))
        {
            return 3;
        }
        else if ((distanceFront > 0.39) & (distanceLeft > 0.5) & (distanceRight < 0.5))
        {
            return 4;
        }
        else if ((distanceFront > 0.39) & (distanceLeft < 0.5) & (distanceRight > 0.5))
        {
            return 5;
        }
        else if ((distanceFront < 0.39) & (distanceLeft > 0.5) & (distanceRight > 0.5))
        {
            return 6;
        }
        else if ((distanceFront > 0.39) & (distanceLeft > 0.5) & (distanceRight > 0.5))
        {
            return 7;
        }
        else
        {
            return 0;
        }
    }

    void goForward(int dis)
    {
        cout << "Going forward..." << endl;
        for (int i = 0; i < dis; i++)
        {
            double leftWheelSensor = getLeftWheelSensor();
            double rightWheelSensor = getRightWheelSensor();

            cout << "Left wheel sensor: " << leftWheelSensor << " | Right wheel sensor: " << rightWheelSensor << endl;

            for (double startLeft = getLeftWheelSensor(); (getLeftWheelSensor() - startLeft) < 11.9;)
            {
                if ((getLeftWheelSensor() - startLeft) >= (5.95) && (getLeftWheelSensor() - startLeft) < (6.1))
                {
                    globalDistance = getDistanceSensors();
                }
                leftMotor->setVelocity(4);
                rightMotor->setVelocity(4);
                step(timeStep); // Ensure the robot steps forward in the simulation
            }

            stopRobot();
        }
    }
    // void turnLeft(double angle)
    // {
    //     const double *initialGyroValues = gyro->getValues();
    //     double initialAngle = initialGyroValues[2];
    //     double targetAngle = initialAngle + angle;

    //     for (; gyro->getValues()[2] < targetAngle; step(timeStep))
    //     {
    //         leftMotor->setVelocity(-1);
    //         rightMotor->setVelocity(1);
    //     }

    //     stopRobot();
    // }

    // void turnRight(double angle)
    // {
    //     stopRobot();
    //     const double *initialGyroValues = gyro->getValues();
    //     double initialAngle = initialGyroValues[2];
    //     double targetAngle = initialAngle - angle;

    //     for (; gyro->getValues()[2] > targetAngle; step(timeStep))
    //     {
    //         cout << "Turning right..." << gyro->getValues()[2] << " " << targetAngle << endl;
    //         leftMotor->setVelocity(1);
    //         rightMotor->setVelocity(-1);
    //     }
    //     cout << "Turned right..." << gyro->getValues()[2] << " " << targetAngle << endl;
    //     stopRobot();
    // }
    double getGyroReading()
    {
        const double *gyroValues = gyro->getValues();
        double angularVelocityZ = gyroValues[2];
        static double angleZ = 0.0;
        angleZ += angularVelocityZ * (timeStep / 1000.0); // Convert timeStep to seconds
        return angleZ;
    }

    void turnLeft()
    {
        double initialLeftWheel = getLeftWheelSensor();
        double targetLeftWheel = initialLeftWheel - 2.241;

        for (; getLeftWheelSensor() > targetLeftWheel; step(timeStep))
        {
            leftMotor->setVelocity(-0.8);
            rightMotor->setVelocity(0.8);
        }
        cout << "Turned left... done" << getLeftWheelSensor() << " " << targetLeftWheel << endl;

        stopRobot();
    }

    void turnRight()
    {
        double initialRightWheel = getRightWheelSensor();
        double targetRightWheel = initialRightWheel - 2.241;

        for (; getRightWheelSensor() > targetRightWheel; step(timeStep))
        {
            leftMotor->setVelocity(0.8);
            rightMotor->setVelocity(-0.8);
        }

        cout << "Turned right... done" << getRightWheelSensor() << " " << targetRightWheel << endl;

        stopRobot();
    }
    void stopRobot()
    {
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    double getDistance(DistanceSensor *ds)
    {
        return ds->getValue();
    }

    // get left wheel sensor
    double getLeftWheelSensor()
    {
        return left_wheel_sensor->getValue();
    }

    // get right wheel sensor
    double getRightWheelSensor()
    {
        return right_wheel_sensor->getValue();
    }

    int calculateCell(double x, double y)
    {
        const double leftX = 1.25;
        const double topY = -1.25;
        const double cellWidth = 0.25;
        const double cellHeight = 0.25;
        const int columns = 10;

        int column = static_cast<int>((leftX - x) / cellWidth) + 1;
        int row = static_cast<int>((y - topY) / cellHeight) + 1;

        if (column < 1 || column > columns || row < 1 || row > columns)
        {
            return -1;
        }

        int cellNumber = (row - 1) * columns + column;
        return cellNumber;
    }

    void run()
    {
        cout << "Starting simulation loop..." << endl;

        globalDistance = getDistanceSensors();
        cout << "Global distance: " << globalDistance << endl;

        // Add a delay of 3 seconds
        for (int i = 0; i < 3000 / timeStep; ++i)
        {
            step(timeStep);
        }

        globalDistance = getDistanceSensors();

        while (step(timeStep) != -1)
        {
            // turnLeft(1.57); // Turn left by 90 degrees (1.57 radians)
            // turnRight(1.57); // Turn right by 90 degrees (1.57 radians)
            // goForward(1);

            int disjunctions = globalDistance;
            cout << "Disjunctions: " << disjunctions << endl;
            if (disjunctions == 1)
            {
                turnLeft();
                stopRobot();
                goForward(1);
            }
            else if (disjunctions == 2)
            {
                goForward(1);
            }
            else if (disjunctions == 3)
            {
                turnLeft();
                stopRobot();
                goForward(1);
            }
            else if (disjunctions == 4)
            {
                turnRight();
                stopRobot();
                goForward(1);
            }
            else if (disjunctions == 5)
            {
                turnLeft();
                stopRobot();
                goForward(1);
            }
            else if (disjunctions == 6)
            {
                goForward(1);
            }
            else if (disjunctions == 7)
            {
                turnRight();
                stopRobot();
                turnRight();
                stopRobot();
                goForward(1);
            }

            // globalDistance = getDistanceSensors();

            double distance_front = getDistance(ds_front);
            double distance_left = getDistance(ds_left);
            double distance_right = getDistance(ds_right);
            // cout << "Distance sensor: " << distance_front << " " << distance_left << " " << distance_right << endl;

            double leftWheelSensor = getLeftWheelSensor();
            double rightWheelSensor = getRightWheelSensor();

            cout << "Left wheel sensor: " << leftWheelSensor << " | Right wheel sensor: " << rightWheelSensor << endl;

            const double *gpsValues = gps->getValues();
            double x = gpsValues[0];
            double y = gpsValues[1];

            int cellNumber = calculateCell(x, y);

            if (cellNumber == -1)
            {
                // cout << "The robot is out of bounds!" << endl;
                // cout << "GPS Coordinates: "
                // << "X = " << gpsValues[0]
                // << ", Y = " << gpsValues[1]
                // << ", Z = " << gpsValues[2]
                // << endl;
            }
            else
            {
                cout << "GPS Coordinates: X = " << x << ", Y = " << y
                     << " | Cell Number: " << cellNumber << endl;
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
    DistanceSensor *ds_left;
    DistanceSensor *ds_right;
    PositionSensor *left_wheel_sensor;
    PositionSensor *right_wheel_sensor;
    Gyro *gyro;
};

int main()
{
    MyRobot robot;
    robot.run();
    return 0;
}
