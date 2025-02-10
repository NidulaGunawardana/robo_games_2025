#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <iostream>
#include <cmath>
#include <iomanip>

using namespace webots;
using namespace std;

struct coordinate
{
    int y;
    int x;
};

struct surroundCoor
{
    struct coordinate N;
    struct coordinate S;
    struct coordinate W;
    struct coordinate E;
};

int orient = 0; // 0: North, 1: East, 2: South, 3: West

const int ROWS = 10;
const int COLUMNS = 10;

int flood[ROWS][COLUMNS] = {
    {9, 10, 11, 16, 17, 18, 19, 20, 21, 24},
    {8, 7, 12, 15, 16, 17, 20, 21, 22, 23},
    {7, 6, 13, 14, 15, 18, 19, 22, 25, 24},
    {6, 5, 0, 1, 2, 5, 6, 23, 24, 25},
    {7, 4, 1, 2, 3, 4, 7, 24, 25, 26},
    {8, 3, 2, 3, 4, 5, 8, 25, 26, 27},
    {5, 4, 3, 10, 7, 6, 9, 10, 27, 28},
    {6, 7, 8, 9, 14, 13, 12, 11, 28, 29},
    {11, 8, 11, 12, 15, 38, 35, 34, 31, 30},
    {10, 9, 10, 13, 16, 37, 36, 33, 32, 33}};

int yellow_flood[ROWS][COLUMNS] = {
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 16, 15, 14, 11, 10, 100, 100, 100},
    {100, 100, 100, 100, 13, 12, 9, 100, 100, 100},
    {100, 100, 100, 100, 100, 100, 8, 100, 100, 100},
    {100, 100, 100, 100, 100, 100, 7, 6, 100, 100},
    {100, 100, 100, 100, 2, 3, 4, 5, 100, 100},
    {100, 100, 100, 100, 1, 100, 100, 100, 100, 100},
    {100, 100, 100, 100, 0, 100, 100, 100, 100, 100}

};

int pink_flood[ROWS][COLUMNS] = {
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 3, 4, 5, 8, 9, 100, 100, 100},
    {100, 100, 2, 100, 6, 7, 10, 100, 100, 100},
    {100, 100, 1, 100, 100, 100, 11, 100, 100, 100},
    {100, 100, 0, 100, 100, 100, 12, 13, 100, 100},
    {100, 100, 100, 100, 17, 16, 15, 14, 100, 100},
    {100, 100, 100, 100, 18, 100, 100, 100, 100, 100},
    {100, 100, 100, 100, 19, 100, 100, 100, 100, 100}};

int brown_flood[ROWS][COLUMNS] = {
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {100, 0, 1, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 2, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100}};

int green_flood[ROWS][COLUMNS] = {
    {19, 18, 17, 12, 11, 10, 9, 8, 7, 100},
    {20, 21, 16, 13, 100, 100, 100, 100, 6, 5},
    {100, 22, 15, 14, 100, 100, 100, 100, 100, 4},
    {100, 23, 100, 100, 100, 100, 100, 100, 100, 3},
    {100, 24, 100, 100, 100, 100, 100, 100, 100, 2},
    {100, 25, 100, 100, 100, 100, 100, 100, 100, 1},
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 0},
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100}};

static coordinate XY;

// Define a custom robot class inheriting from the Webots Robot class
class MyRobot : public Robot
{
public:
    MyRobot()
    {
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
        imu = getInertialUnit("imu");

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
        imu->enable(timeStep);

        // Set motors to velocity control mode with initial velocity set to 0
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    string getDirection()
    {
        const float MPI = 3.14;

        // Get the IMU device
        if (!imu)
        {
            cerr << "IMU device not found!" << endl;
            return "Error: IMU not found";
        }

            // Get roll, pitch, yaw from the IMU
            const double *rpy = imu->getRollPitchYaw();
            double yaw = rpy[2];

            // Normalize yaw to [0, 2*PI] for easier handling
            if (yaw < 0)
            {
                yaw += 2 * MPI;
            }

            // Determine direction based on yaw
            if ((yaw >= 7 * M_PI / 4 || yaw < M_PI / 4))
            {
                return "East"; // East
            }
            else if (yaw >= M_PI / 4 && yaw < 3 * M_PI / 4)
            {
                return "North"; // North
            }
            else if (yaw >= 3 * M_PI / 4 && yaw < 5 * M_PI / 4)
            {
                return "West"; // West
            }
            else if (yaw >= 5 * M_PI / 4 && yaw < 7 * M_PI / 4)
            {
                return "South"; // South
            }
            else
            {
                return "Error: Unable to determine direction";
            }

    }

    // Method to read distance sensors and determine the robot's surroundings
    int getDistanceSensorsFirstCell()
    {

        string direction_init = getDirection();
        if (direction_init.find("Error") == string::npos)
        {
            cout << "Direction: " << direction_init << endl;
        }
        else
        {
            cerr << direction_init << endl;
        }

        double distanceFront = getDistance(ds_front);
        double distanceLeft = getDistance(ds_left);
        double distanceRight = getDistance(ds_right);

        cout << "Distance sensor values: Front=" << distanceFront
             << ", Left=" << distanceLeft
             << ", Right=" << distanceRight << endl;

        // Evaluate sensor readings and return corresponding identifier
        if (distanceFront > 0.39 && distanceLeft < 0.7 && distanceRight < 0.7)
        {
            cout << "Wall Ahead" << endl;
            return 1;
        }
        else if (distanceFront < 0.39 && distanceLeft > 0.7 && distanceRight < 0.7)
        {
            cout << "Wall Left" << endl;
            return 2;
        }
        else if (distanceFront < 0.39 && distanceLeft < 0.7 && distanceRight > 0.7)
        {
            cout << "Wall Right" << endl;
            return 3;
        }
        else if (distanceFront > 0.39 && distanceLeft > 0.7 && distanceRight < 0.7)
        {
            cout << "Wall Ahead and Left" << endl;
            return 4;
        }
        else if (distanceFront > 0.39 && distanceLeft < 0.7 && distanceRight > 0.7)
        {
            cout << "Wall Ahead and Right" << endl;
            return 5;
        }
        else if (distanceFront < 0.39 && distanceLeft > 0.7 && distanceRight > 0.7)
        {
            cout << "Wall Left and Right" << endl;
            return 6;
        }
        else if (distanceFront > 0.39 && distanceLeft > 0.7 && distanceRight > 0.7)
        {
            cout << "Wall Ahead, Left and Right" << endl;
            return 7;
        }
        else
        {
            cout << "No Walls Around" << endl;
            return 0; // No significant obstacles detected
        }
    }

    int getDistanceSensors()
    {
        double distanceFront = getDistance(ds_front);
        double distanceLeft = getDistance(ds_left);
        double distanceRight = getDistance(ds_right);

        //out << "Distance sensor values: Front=" << distanceFront << ", Left=" << distanceLeft << ", Right=" << distanceRight << endl;

        // Evaluate sensor readings and return corresponding identifier
        if (distanceFront > 0.39 && distanceLeft < 0.5 && distanceRight < 0.5)
        {
            cout << "Wall Ahead" << endl;
            return 1;
        }
        else if (distanceFront < 0.39 && distanceLeft > 0.5 && distanceRight < 0.5)
        {
            cout << "Wall Left" << endl;
            return 2;
        }
        else if (distanceFront < 0.39 && distanceLeft < 0.5 && distanceRight > 0.5)
        {
            cout << "Wall Right" << endl;
            return 3;
        }
        else if (distanceFront > 0.39 && distanceLeft > 0.5 && distanceRight < 0.5)
        {
            cout << "Wall Ahead and Left" << endl;
            return 4;
        }
        else if (distanceFront > 0.39 && distanceLeft < 0.5 && distanceRight > 0.5)
        {
            cout << "Wall Ahead and Right" << endl;
            return 5;
        }
        else if (distanceFront < 0.39 && distanceLeft > 0.5 && distanceRight > 0.5)
        {
            cout << "Wall Left and Right" << endl;
            return 6;
        }
        else if (distanceFront > 0.39 && distanceLeft > 0.5 && distanceRight > 0.5)
        {
            cout << "Wall Ahead, Left and Right" << endl;
            return 7;
        }
        else
        {
            cout << "No Walls Around" << endl;
            return 0; // No significant obstacles detected
        }
    }

    // Move the robot forward by a specified distance (in simulation steps)
    void goForward(int steps)
    {
        cout << "Going forward for " << steps << " steps..." << endl;

        // PID controller parameters
        double Kp = 0.03;
        double Ki = 0.0;
        double Kd = 0.01;
        double previousError = 0.0;
        double integral = 0.0;

        for (int i = 0; i < steps; i++)
        {
            double initialLeftPosition = getLeftWheelSensor();

            // Move forward until the left wheel rotates a specific distance
            while ((initialLeftPosition - getLeftWheelSensor()) < 12.5)
            {
                // Update global distance sensor readings midway
                if ((initialLeftPosition - getLeftWheelSensor()) >= 5.95 && (getLeftWheelSensor() - initialLeftPosition) < 6.1)
                {
                    wall_arrangement = getDistanceSensors();
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

                leftMotor->setVelocity(-leftSpeed);
                rightMotor->setVelocity(-rightSpeed);
                step(timeStep); // Advance simulation
            }

            stopRobot(); // Stop robot after moving the required distance
        }
    }

    // Turn the robot left by 90 degrees using wheel encoders
    void turnLeft()
    {
        double initialLeftWheel = getLeftWheelSensor();
        double targetLeftWheel = initialLeftWheel + 2.241; // Approx. rotation for 90 degrees

        // Rotate until the left wheel reaches the target position
        while (getLeftWheelSensor() < targetLeftWheel)
        {
            leftMotor->setVelocity(0.8);
            rightMotor->setVelocity(-0.8);
            step(timeStep);
        }
        stopRobot();

        // update orientation
        orient -= 1;
        if (orient == -1)
        {
            orient = 3;
        }
    }

    // Turn the robot right by 90 degrees using wheel encoders
    void turnRight()
    {
        double initialRightWheel = getRightWheelSensor();
        double targetRightWheel = initialRightWheel + 2.241; // Approx. rotation for 90 degrees

        // Rotate until the right wheel reaches the target position
        while (getRightWheelSensor() < targetRightWheel)
        {
            leftMotor->setVelocity(-0.8);
            rightMotor->setVelocity(0.8);
            step(timeStep);
        }
        stopRobot();

        orient += 1;
        if (orient == 4)
        {
            orient = 0;
        }
    }

    // Align the robot with the wall using the front distance sensor
    void align_wall()
    {
        double distanceFront = getDistance(ds_front);

        while (distanceFront < 0.89 || distanceFront > 0.9)
        {
            if (distanceFront < 0.89)
            {
                leftMotor->setVelocity(-1);
                rightMotor->setVelocity(-1);
            }
            else if (distanceFront > 0.9)
            {
                leftMotor->setVelocity(1);
                rightMotor->setVelocity(1);
            }
            step(timeStep);
            distanceFront = getDistance(ds_front);
        }
        stopRobot();
    }

    // Make the robot parallel to the wall using front-left and front-right distance sensors
    void parallel_wall()
    {
        double distanceLeft = getDistance(ds_front_left);
        double distanceRight = getDistance(ds_front_right);
        double distanceDifference = distanceLeft - distanceRight;

        while (fabs(distanceDifference) > 0.001)
        { // Continue until the difference is close to zero
            if (distanceDifference > 0.01)
            {
                leftMotor->setVelocity(0.5);
                rightMotor->setVelocity(-0.5);
            }
            else if (distanceDifference < -0.01)
            {
                leftMotor->setVelocity(-0.5);
                rightMotor->setVelocity(0.5);
            }
            step(timeStep);
            distanceLeft = getDistance(ds_front_left);
            distanceRight = getDistance(ds_front_right);
            distanceDifference = distanceLeft - distanceRight;
        }
        stopRobot();
    }

    // Stop the robot by setting motor velocities to zero
    void stopRobot()
    {
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    // Get distance reading from a specific distance sensor
    double getDistance(DistanceSensor *ds)
    {
        return ds->getValue();
    }

    // Get the current reading of the left wheel position sensor
    double getLeftWheelSensor()
    {
        return left_wheel_sensor->getValue();
    }

    // Get the current reading of the right wheel position sensor
    double getRightWheelSensor()
    {
        return right_wheel_sensor->getValue();
    }

    // Calculate the current grid cell based on GPS coordinates
    struct coordinate calculateCell(double gps_x, double gps_y)
    {
        const double leftX = 1.25;
        const double topY = -1.25;
        const double cellWidth = 0.25;
        const double cellHeight = 0.25;
        const int columns = 9;
        struct coordinate XY;

        int column = static_cast<int>((leftX - gps_x) / cellWidth);
        int row = static_cast<int>((gps_y - topY) / cellHeight);

        if (column < 0 || column > columns || row < 0 || row > columns)
        {
            return {-1, -1}; // Out of bounds
        }

        column = columns - column; // Reverse column order
        XY.x = column;
        XY.y = row;
        return XY;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Code By Mihiruth
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    int current_cell(struct coordinate p)
    { // Returns the flood cell number of the current cell
        int cell_no = 0;
        if (p.x >= 0 && p.x < ROWS && p.y >= 0 && p.y < COLUMNS)
        {
            cell_no = flood[p.y][p.x];
        }
        else
        {
            p.x = -1;
            p.y = -1;
            // Handle out-of-bounds case
            // throw an error or assign a default value
            cout << "Flood Out of bounds" << endl;
        }

        return cell_no;
    }

    struct surroundCoor getSurrounds(struct coordinate p)
    {
        struct surroundCoor surCoor;
        surCoor.N.x = p.x;
        surCoor.N.y = p.y + 1;

        surCoor.S.x = p.x;
        surCoor.S.y = p.y - 1;

        surCoor.W.x = p.x - 1;
        surCoor.W.y = p.y;

        surCoor.E.x = p.x + 1;
        surCoor.E.y = p.y;

        if (surCoor.N.x >= ROWS)
        {
            surCoor.N.x = -1;
        }
        if (surCoor.W.y >= COLUMNS)
        {
            surCoor.W.y = -1;
        }

        return surCoor;
    }

    bool compareCoordinates(struct coordinate a, struct coordinate b)
    {
        return (a.x == b.x) && (a.y == b.y);
    }

    bool isAccessible(int wall_arrangement, struct coordinate current_coor, struct coordinate next_coor)
    {
        struct surroundCoor surCoor = getSurrounds(current_coor);

        switch (orient)
        {
        case 0:
            switch (wall_arrangement)
            {
            case 1: // Wall on the front
                if (compareCoordinates(next_coor, surCoor.N))
                {
                    return false;
                }
                break;

            case 2: // Wall on the left
                if (compareCoordinates(next_coor, surCoor.W))
                {
                    return false;
                }
                break;

            case 3: // Wall on the right
                if (compareCoordinates(next_coor, surCoor.E))
                {
                    return false;
                }
                break;

            case 4: // Walls on the front and left
                if (compareCoordinates(next_coor, surCoor.N) ||
                    compareCoordinates(next_coor, surCoor.W))
                {
                    return false;
                }
                break;

            case 5: // Walls on the front and right
                if (compareCoordinates(next_coor, surCoor.N) ||
                    compareCoordinates(next_coor, surCoor.E))
                {
                    return false;
                }
                break;

            case 6: // Walls on the left and right
                if (compareCoordinates(next_coor, surCoor.W) ||
                    compareCoordinates(next_coor, surCoor.E))
                {
                    return false;
                }
                break;

            case 7: // Walls on all sides except bottom
                if (compareCoordinates(next_coor, surCoor.N) || compareCoordinates(next_coor, surCoor.W) || compareCoordinates(next_coor, surCoor.E))
                {
                    return false;
                }
                break;

            default:
                return true;
            }
            break;

        case 1:

            switch (wall_arrangement)
            {
            case 1: // Wall on the front
                if (compareCoordinates(next_coor, surCoor.E))
                {
                    return false;
                }
                break;

            case 2: // Wall on the left
                if (compareCoordinates(next_coor, surCoor.N))
                {
                    return false;
                }
                break;

            case 3: // Wall on the right
                if (compareCoordinates(next_coor, surCoor.S))
                {
                    return false;
                }
                break;

            case 4: // Walls on the front and left
                if (compareCoordinates(next_coor, surCoor.E) ||
                    compareCoordinates(next_coor, surCoor.N))
                {
                    return false;
                }
                break;

            case 5: // Walls on the front and right
                if (compareCoordinates(next_coor, surCoor.E) ||
                    compareCoordinates(next_coor, surCoor.S))
                {
                    return false;
                }
                break;

            case 6: // Walls on the left and right
                if (compareCoordinates(next_coor, surCoor.N) ||
                    compareCoordinates(next_coor, surCoor.S))
                {
                    return false;
                }
                break;

            case 7: // Walls on all sides except bottom
                if (compareCoordinates(next_coor, surCoor.E) || compareCoordinates(next_coor, surCoor.N) || compareCoordinates(next_coor, surCoor.S))
                {
                    return false;
                }
                break;

            default:
                return true;
            }
            break;

        case 2:

            switch (wall_arrangement)
            {
            case 1: // Wall on the front
                if (compareCoordinates(next_coor, surCoor.S))
                {
                    return false;
                }
                break;

            case 2: // Wall on the left
                if (compareCoordinates(next_coor, surCoor.E))
                {
                    return false;
                }
                break;

            case 3: // Wall on the right
                if (compareCoordinates(next_coor, surCoor.W))
                {
                    return false;
                }
                break;

            case 4: // Walls on the front and left
                if (compareCoordinates(next_coor, surCoor.S) ||
                    compareCoordinates(next_coor, surCoor.E))
                {
                    return false;
                }
                break;

            case 5: // Walls on the front and right
                if (compareCoordinates(next_coor, surCoor.S) ||
                    compareCoordinates(next_coor, surCoor.W))
                {
                    return false;
                }
                break;

            case 6: // Walls on the left and right
                if (compareCoordinates(next_coor, surCoor.E) ||
                    compareCoordinates(next_coor, surCoor.W))
                {
                    return false;
                }
                break;

            case 7: // Walls on all sides except bottom
                if (compareCoordinates(next_coor, surCoor.S) || compareCoordinates(next_coor, surCoor.E) || compareCoordinates(next_coor, surCoor.W))
                {
                    return false;
                }
                break;

            default:
                return true;
            }
            break;

        case 3:

            switch (wall_arrangement)
            {
            case 1: // Wall on the front
                if (compareCoordinates(next_coor, surCoor.W))
                {
                    return false;
                }
                break;

            case 2: // Wall on the left
                if (compareCoordinates(next_coor, surCoor.S))
                {
                    return false;
                }
                break;

            case 3: // Wall on the right
                if (compareCoordinates(next_coor, surCoor.N))
                {
                    return false;
                }
                break;

            case 4: // Walls on the front and left
                if (compareCoordinates(next_coor, surCoor.W) ||
                    compareCoordinates(next_coor, surCoor.S))
                {
                    return false;
                }
                break;

            case 5: // Walls on the front and right
                if (compareCoordinates(next_coor, surCoor.W) ||
                    compareCoordinates(next_coor, surCoor.N))
                {
                    return false;
                }
                break;

            case 6: // Walls on the left and right
                if (compareCoordinates(next_coor, surCoor.S) ||
                    compareCoordinates(next_coor, surCoor.N))
                {
                    return false;
                }
                break;

            case 7: // Walls on all sides except bottom
                if (compareCoordinates(next_coor, surCoor.W) || compareCoordinates(next_coor, surCoor.S) || compareCoordinates(next_coor, surCoor.N))
                {
                    return false;
                }
                break;

            default:
                return true;
            }
            break;

        default:
            break;
        }

        return true; // If no case matches
    }

    char toMove(struct coordinate p, int wall_arrangement)
    {
        struct surroundCoor surCoor = getSurrounds(p);
        int min = current_cell(p);
        cout << "Current cell no: " << min << endl;
        struct coordinate next_coor = p;

        int directionsLooked = 0;

        while (true)
        {
            bool exitLoop = false; // Flag to exit the while loop

            switch (directionsLooked)
            {
            case 0:
                if ((surCoor.N.x < 10 && surCoor.N.y < 10) && (surCoor.N.x >= 0 && surCoor.N.y >= 0))
                {
                    int cell_no = current_cell(surCoor.N);
                    cout << "Looking N: " << cell_no << endl;
                    if ((cell_no == min-1) && isAccessible(wall_arrangement, p, surCoor.N))
                    {
                        min = cell_no;
                        next_coor = surCoor.N;
                        exitLoop = true; // Set the flag to exit the while loop
                        break;           // Break the switch
                    }
                }
                directionsLooked++;
                break;

            case 1:
                if ((surCoor.W.x < 10 && surCoor.W.y < 10) && (surCoor.W.x >= 0 && surCoor.W.y >= 0))
                {
                    int cell_no = current_cell(surCoor.W);
                    cout << "Looking W: " << cell_no << endl;
                    if ((cell_no == min-1) && isAccessible(wall_arrangement, p, surCoor.W))
                    {
                        min = cell_no;
                        next_coor = surCoor.W;
                        exitLoop = true; // Set the flag to exit the while loop
                        break;           // Break the switch
                    }
                }
                directionsLooked++;
                break;

            case 3:
                if ((surCoor.S.x < 10 && surCoor.S.y < 10) && (surCoor.S.x >= 0 && surCoor.S.y >= 0))
                {
                    int cell_no = current_cell(surCoor.S);
                    cout << "Looking S: " << cell_no << endl;
                    if ((cell_no == min-1) && isAccessible(wall_arrangement, p, surCoor.S))
                    {
                        min = cell_no;
                        next_coor = surCoor.S;
                        exitLoop = true; // Set the flag to exit the while loop
                        break;           // Break the switch
                    }
                }
                directionsLooked++;
                break;

            case 2:
                if ((surCoor.E.x < 10 && surCoor.E.y < 10) && (surCoor.E.x >= 0 && surCoor.E.y >= 0))
                {
                    int cell_no = current_cell(surCoor.E);
                    cout << "Looking E: " << cell_no << endl;
                    if ((cell_no == min-1) && isAccessible(wall_arrangement, p, surCoor.E))
                    {
                        min = cell_no;
                        next_coor = surCoor.E;
                        exitLoop = true; // Set the flag to exit the while loop
                        break;           // Break the switch
                    }
                }
                directionsLooked++;
                break;

            default:
                break;
            }

            if (exitLoop)
            { // Exit the while loop if the flag is set
                break;
            }
        }

        switch (orient)
        {
        case 0:
            if (compareCoordinates(next_coor, surCoor.N))
            {
                return 'F'; // Forward
            }
            else if (compareCoordinates(next_coor, surCoor.W))
            {
                return 'L'; // Left
            }
            else if (compareCoordinates(next_coor, surCoor.S))
            {
                return 'B'; // Backward
            }
            else if (compareCoordinates(next_coor, surCoor.E))
            {
                return 'R'; // Right
            }
            break;

        case 1:
            if (compareCoordinates(next_coor, surCoor.E))
            {
                return 'F'; // Forward
            }
            else if (compareCoordinates(next_coor, surCoor.N))
            {
                return 'L'; // Left
            }
            else if (compareCoordinates(next_coor, surCoor.W))
            {
                return 'B'; // Backward
            }
            else if (compareCoordinates(next_coor, surCoor.S))
            {
                return 'R'; // Right
            }
            break;

        case 2:
            if (compareCoordinates(next_coor, surCoor.S))
            {
                return 'F'; // Forward
            }
            else if (compareCoordinates(next_coor, surCoor.E))
            {
                return 'L'; // Left
            }
            else if (compareCoordinates(next_coor, surCoor.N))
            {
                return 'B'; // Backward
            }
            else if (compareCoordinates(next_coor, surCoor.W))
            {
                return 'R'; // Right
            }
            break;

        case 3:
            if (compareCoordinates(next_coor, surCoor.W))
            {
                return 'F'; // Forward
            }
            else if (compareCoordinates(next_coor, surCoor.S))
            {
                return 'L'; // Left
            }
            else if (compareCoordinates(next_coor, surCoor.E))
            {
                return 'B'; // Backward
            }
            else if (compareCoordinates(next_coor, surCoor.N))
            {
                return 'R'; // Right
            }
            break;

        default:
            break;
        }

        return 'X'; // No valid move
    }

    void moveFirstCell()
    {

        cout << "Moving in the First Cell......" << endl;

        // Initial delay (3 seconds)
        for (int i = 0; i < 1000 / timeStep; ++i)
        {
            step(timeStep);
        }

        string facing_direction = getDirection();
        cout << "Initial Direction : " << facing_direction << endl;

        if (facing_direction == "North") {
            orient = 0;
        } else if (facing_direction == "East") {
            orient = 1;
        } else if (facing_direction == "South") {
            orient = 2;
        } else if (facing_direction == "West") {
            orient = 3;
        } else {
            // Handle invalid input
            orient = -1; // Example: -1 for invalid orientation
        }

        cout << "Turning into north " << orient << endl;

        if(orient == 0){
            cout << "Robot is Good to go" << endl;
        } else if(orient == 1){
            turnLeft();
        } else if(orient == 2){
            turnLeft();
            turnLeft();
        } else if(orient == 3){
            turnRight();
        }

        


        // Initial delay (3 seconds)
        for (int i = 0; i < 1000 / timeStep; ++i)
        {
            step(timeStep);
        }

        char direction = 'X';
        wall_arrangement = getDistanceSensorsFirstCell();

        double distanceFront = getDistance(ds_front);
        cout << "Distance sensor values: Front=" << distanceFront << endl;

        // Log robot's position and cell
        const double *gpsValues = gps->getValues();
        double gps_x = gpsValues[0];
        double gps_y = gpsValues[1];

        XY = calculateCell(gps_x, gps_y);
        int x = XY.x;
        int y = XY.y;

        if (x != -1 && y != -1)
        {
            cout << "GPS Coordinates: X=" << gps_x << ", Y=" << gps_y << " | X =" << x << ", Y =" << y << endl;
        }

        direction = toMove(XY, wall_arrangement);
        cout << "Direction: " << direction << endl;

        switch (direction)
        {
        case 'F':
            // goForward(1);
            break;

        case 'L':
            turnLeft();
            // goForward(1);
            break;

        case 'R':
            turnRight();
            // goForward(1);
            break;

        case 'B':
            turnRight();
            turnRight();
            // goForward(1);
            break;

        default:
            break;
        }

        cout << "First Cell Movement Completed......" << endl;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////

    // Main simulation loop
    void run()
    {

        int color_state = 0; // 0: if red color haven't been detected, 1: if red color detected
        char direction = 'X';

        cout << "Starting simulation loop..." << endl;

        // Initial delay (3 seconds)
        for (int i = 0; i < 1000 / timeStep; ++i)
        {
            step(timeStep);
        }

        ////////////////////////////////////
        // Add here code to get the direction//
        // Example :
        // facing_irection = getDirection()
        // Cout<< facing_direction << endl

        /////////////////////////////////

        // Facing North

        // cout << "Turning into north " << orient << endl;

        // if(orient == 0){
        //     cout << "Robot is Good to go" << endl;
        // } else if(orient == 1){
        //     turnLeft();
        // } else if(orient == 2){
        //     turnLeft();
        //     turnLeft();
        // } else if(orient == 3){
        //     turnRight();
        // }

        for (int i = 0; i < 1000 / timeStep; ++i)
        {
            step(timeStep);
        }

        // Get initial distance sensor readings
        wall_arrangement = getDistanceSensors();

        while (step(timeStep) != -1)
        {
            // Print front distance sensor value
            double distanceFront = getDistance(ds_front);
            cout << "Distance sensor values: Front=" << distanceFront << endl;

            // Log robot's position and cell
            const double *gpsValues = gps->getValues();
            double gps_x = gpsValues[0];
            double gps_y = gpsValues[1];

            XY = calculateCell(gps_x, gps_y);
            int x = XY.x;
            int y = XY.y;

            if (x != -1 && y != -1)
            {
                cout << "GPS Coordinates: X=" << gps_x << ", Y=" << gps_y << " | X =" << x << ", Y =" << y << endl;
            }

            switch (color_state)
            {

            case 0:

                if (current_cell(XY) == 0)
                {
                    color_state = 1;
                    cout << "Red color detected" << endl;
                    cout << "Current Cell is " << current_cell(XY) << "  Cell Coordinates :" << XY.x << "," << XY.y << endl;
                    // Copy yellow_flood to flood
                    for (int i = 0; i < ROWS; ++i)
                    {
                        for (int j = 0; j < COLUMNS; ++j)
                        {
                            flood[i][j] = yellow_flood[i][j];
                        }
                    }

                    cout << "Arrays Copied Successfully. Current Number is " << yellow_flood[XY.y][XY.x] << endl;
                    break;
                }

                direction = toMove(XY, wall_arrangement);
                cout << "Direction: " << direction << endl;

                cout << "Is north accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).N) << endl;
                cout << "Is south accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).S) << endl;
                cout << "Is west accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).W) << endl;
                cout << "Is east accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).E) << endl;

                switch (direction)
                {
                case 'F':
                    goForward(1);
                    break;

                case 'L':
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnLeft();
                    goForward(1);
                    break;

                case 'R':
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnRight();
                    goForward(1);
                    break;

                case 'B':
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnRight();
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnRight();
                    goForward(1);
                    break;

                default:
                    break;
                }

                break;

            case 1: // Moving to Yellow Color

                // cout<<"Moving for yellow"<<endl;

                // for (int i = 0; i < 1000 / timeStep; ++i) {
                //     step(timeStep);
                // }

                // cout << "Current Coordinates : " << XY.x << "," << XY.y << endl;

                if (current_cell(XY) == 0)
                {
                    color_state = 2;
                    cout << "Yellow color detected" << endl;
                    cout << "Current Cell is " << current_cell(XY) << "  Cell Coordinates :" << XY.x << "," << XY.y << endl;
                    // Copy yellow_flood to flood
                    for (int i = 0; i < ROWS; ++i)
                    {
                        for (int j = 0; j < COLUMNS; ++j)
                        {
                            flood[i][j] = pink_flood[i][j];
                        }
                    }

                    cout << "Arrays Copied Successfully. Current Number is " << flood[XY.y][XY.x] << endl;
                    break;
                }

                direction = toMove(XY, wall_arrangement);
                cout << "Direction: " << direction << endl;

                cout << "Is north accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).N) << endl;
                cout << "Is south accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).S) << endl;
                cout << "Is west accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).W) << endl;
                cout << "Is east accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).E) << endl;

                switch (direction)
                {
                case 'F':
                    goForward(1);
                    break;

                case 'L':
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnLeft();
                    goForward(1);
                    break;

                case 'R':
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnRight();
                    goForward(1);
                    break;

                case 'B':
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnRight();
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnRight();
                    goForward(1);
                    break;

                default:
                    break;
                }

                break;

            case 2: // Moving to Pink Color

                // for (int i = 0; i < 1000 / timeStep; ++i) {
                //     step(timeStep);
                // }

                // cout << "Current Coordinates : " << XY.x << "," << XY.y << endl;

                if (current_cell(XY) == 0)
                {
                    color_state = 3;
                    cout << "Pink color detected" << endl;
                    cout << "Current Cell is " << current_cell(XY) << "  Cell Coordinates :" << XY.x << "," << XY.y << endl;
                    // Copy yellow_flood to flood
                    for (int i = 0; i < ROWS; ++i)
                    {
                        for (int j = 0; j < COLUMNS; ++j)
                        {
                            flood[i][j] = brown_flood[i][j];
                        }
                    }

                    cout << "Arrays Copied Successfully. Current Number is " << flood[XY.y][XY.x] << endl;
                    break;
                }

                direction = toMove(XY, wall_arrangement);
                cout << "Direction: " << direction << endl;

                cout << "Is north accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).N) << endl;
                cout << "Is south accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).S) << endl;
                cout << "Is west accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).W) << endl;
                cout << "Is east accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).E) << endl;

                switch (direction)
                {
                case 'F':
                    goForward(1);
                    break;

                case 'L':
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnLeft();
                    goForward(1);
                    break;

                case 'R':
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnRight();
                    goForward(1);
                    break;

                case 'B':
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnRight();
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnRight();
                    goForward(1);
                    break;

                default:
                    break;
                }

                break;

            case 3: // Moving to Brown Color

                // for (int i = 0; i < 1000 / timeStep; ++i) {
                //     step(timeStep);
                // }

                // cout << "Current Coordinates : " << XY.x << "," << XY.y << endl;

                if (current_cell(XY) == 0)
                {
                    color_state = 4;
                    cout << "Pink color detected" << endl;
                    cout << "Current Cell is " << current_cell(XY) << "  Cell Coordinates :" << XY.x << "," << XY.y << endl;
                    // Copy yellow_flood to flood
                    for (int i = 0; i < ROWS; ++i)
                    {
                        for (int j = 0; j < COLUMNS; ++j)
                        {
                            flood[i][j] = green_flood[i][j];
                        }
                    }

                    cout << "Arrays Copied Successfully. Current Number is " << flood[XY.y][XY.x] << endl;
                    break;
                }

                direction = toMove(XY, wall_arrangement);
                cout << "Direction: " << direction << endl;

                cout << "Is north accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).N) << endl;
                cout << "Is south accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).S) << endl;
                cout << "Is west accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).W) << endl;
                cout << "Is east accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).E) << endl;

                switch (direction)
                {
                case 'F':
                    goForward(1);
                    break;

                case 'L':
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnLeft();
                    goForward(1);
                    break;

                case 'R':
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnRight();
                    goForward(1);
                    break;

                case 'B':
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnRight();
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnRight();
                    goForward(1);
                    break;

                default:
                    break;
                }

                break;

            case 4: // Moving to Green Color

                // for (int i = 0; i < 1000 / timeStep; ++i) {
                //     step(timeStep);
                // }

                // cout << "Current Coordinates : " << XY.x << "," << XY.y << endl;

                if (current_cell(XY) == 0)
                {
                    color_state = 5;
                    cout << "Green color detected" << endl;
                    cout << "Current Cell is " << current_cell(XY) << "  Cell Coordinates :" << XY.x << "," << XY.y << endl;
                    // //Copy yellow_flood to flood
                    // for (int i = 0; i < ROWS; ++i) {
                    //     for (int j = 0; j < COLUMNS; ++j) {
                    //         flood[i][j] = brown_flood[i][j];

                    //     }
                    // }

                    cout << "Arrays Copied Successfully. Current Number is " << flood[XY.y][XY.x] << endl;
                    break;
                }

                direction = toMove(XY, wall_arrangement);
                cout << "Direction: " << direction << endl;

                cout << "Is north accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).N) << endl;
                cout << "Is south accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).S) << endl;
                cout << "Is west accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).W) << endl;
                cout << "Is east accessible: " << isAccessible(wall_arrangement, XY, getSurrounds(XY).E) << endl;

                switch (direction)
                {
                case 'F':
                    goForward(1);
                    break;

                case 'L':
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnLeft();
                    goForward(1);
                    break;

                case 'R':
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnRight();
                    goForward(1);
                    break;

                case 'B':
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnRight();
                    if (getDistance(ds_front) > 0.39)
                    {
                        align_wall();
                        parallel_wall();
                    }
                    turnRight();
                    goForward(1);
                    break;

                default:
                    break;
                }

                break;

            case 5:

                cout << "All colors detected. Mission Completed" << endl;
                break;

            default:
                break;
            }
        }
    };

private:
    int timeStep;
    int wall_arrangement;
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
    InertialUnit *imu;
};

// Entry point of the program
int main()
{
    MyRobot robot; // Create an instance of the robot
    robot.moveFirstCell();
    robot.run(); // Start the simulation loop
    return 0;
}
