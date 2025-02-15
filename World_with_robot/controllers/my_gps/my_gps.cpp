#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <iostream>
#include <cmath>
#include <iomanip>
#include <memory>
#include <stack>
#include <vector>
#include <string>
#include <queue>

#define GREEN_THRESHOLD 80        // Minimum green intensity for detection
#define GREEN_AREA_THRESHOLD 0.06 // 8% of the image must be green to trigger detection

using namespace webots;
using namespace std;

/**
 * Function to process the camera image, detect green areas, and display the binary mask.
 * @param camera - Pointer to the Webots camera.
 * @param display - Pointer to the Webots display.
 * @return true if a large green area is detected, false otherwise.
 */

struct coordinate
{
    int y;
    int x;
};

// Define known floor colors
struct Color
{
    int r, g, b;
    string name;
};

// Define RGB ranges for each color
struct ColorRange
{
    int r_min, r_max;
    int g_min, g_max;
    int b_min, b_max;
    string name;
};

// Predefined color ranges
const ColorRange COLORS[] = {
    {220, 245, 220, 245, 140, 170, "Yellow"}, // Yellow (234, 235, 156)
    {220, 245, 90, 130, 20, 50, "Orange"},    // Orange (233, 109, 35)
    {220, 245, 20, 50, 20, 50, "Red"}         // Red (233, 32, 34)
};

// Default Webots floor color (ignored)
const ColorRange DEFAULT_FLOOR = {160, 190, 120, 150, 90, 120, "Default"};

struct surroundCoor
{
    struct coordinate N;
    struct coordinate S;
    struct coordinate W;
    struct coordinate E;
};

int orient = 0; // 0: North, 1: East, 2: South, 3: West

const int ROWS = 20;
const int COLUMNS = 20;

bool visited[ROWS][COLUMNS] = {false};
struct coordinate greenCells[3];

int greenptr = 0;

int cells[ROWS][COLUMNS] = {
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}};

// Cell colors => -1: Default, 1: Yellow, 2: Orange, 3: Red
int cell_colors[ROWS][COLUMNS] = {
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}};

int flood[ROWS][COLUMNS];

// std::stack<struct coordinate> location_stack;
// std::stack<struct coordinate> branch_stack;

auto location_stack = std::make_unique<std::stack<struct coordinate, std::vector<struct coordinate>>>();
auto branch_stack = std::make_unique<std::stack<struct coordinate, std::vector<struct coordinate>>>();

static coordinate XY;
static coordinate XY_prev;

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
        camera = getCamera("camera");         // Main camera
        floor_camera = getCamera("camera_2"); // Floor camera
        display = getDisplay("display");

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
        camera->enable(timeStep);
        floor_camera->enable(timeStep);

        // Set motors to velocity control mode with initial velocity set to 0
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    /**
     * Function to process the camera image, detect green areas, and display the binary mask.
     * @param camera - Pointer to the Webots camera.
     * @param display - Pointer to the Webots display.
     * @return true if a large green area is detected, false otherwise.
     */
    bool processVision(Camera *camera, Display *display)
    {
        if (!camera || !display)
        {
            cerr << "ERROR: Camera or Display not initialized!" << endl;
            return false;
        }

        int width = camera->getWidth();
        int height = camera->getHeight();

        // Create an empty image buffer
        unsigned char *imageBuffer = new unsigned char[3 * width * height];

        // Get the camera image
        const unsigned char *image = camera->getImage();
        if (!image)
        {
            cerr << "ERROR: Failed to get camera image!" << endl;
            delete[] imageBuffer;
            return false;
        }

        int green_count = 0;
        int total_pixels = width * height;

        // Process Image: Detect Green & Create Binary Mask
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                int index = (y * width + x) * 4; // BGRA format (4 bytes per pixel)

                int r = image[index + 2]; // Red
                int g = image[index + 1]; // Green
                int b = image[index];     // Blue

                // Check if green is dominant
                bool isGreen = (g > GREEN_THRESHOLD && g / 2 > r && g / 2 > b);
                if (isGreen)
                    green_count++;

                // Create binary mask (Green -> Green, Others -> Black)
                imageBuffer[3 * (y * width + x)] = isGreen ? 0 : 0;       // Red
                imageBuffer[3 * (y * width + x) + 1] = isGreen ? 255 : 0; // Green
                imageBuffer[3 * (y * width + x) + 2] = isGreen ? 0 : 0;   // Blue
            }
        }

        // Display the Binary Green Mask
        ImageRef *image_ref = display->imageNew(width, height, imageBuffer, Display::RGB);
        display->imagePaste(image_ref, 0, 0, false);
        display->imageDelete(image_ref);

        delete[] imageBuffer; // Free memory

        // Check if green area is significant
        return (static_cast<double>(green_count) / total_pixels > GREEN_AREA_THRESHOLD);
    }

    /**
     * Function to detect floor color based on predefined RGB ranges.
     * @param camera - Pointer to the Webots camera.
     * @return The detected color name (Yellow, Orange, Red) or an empty string if default.
     */
    char getFloorColor(Camera *camera)
    {
        if (!camera)
            return 'E';

        const unsigned char *image = camera->getImage();
        if (!image)
            return 'E';

        int width = camera->getWidth();
        int height = camera->getHeight();

        // Take the center pixel as the reference
        int x = width / 2;
        int y = height / 2;
        int index = (y * width + x) * 4; // BGRA format

        int r = image[index + 2]; // Red
        int g = image[index + 1]; // Green
        int b = image[index];     // Blue

        // Ignore default floor color
        if (r >= DEFAULT_FLOOR.r_min && r <= DEFAULT_FLOOR.r_max &&
            g >= DEFAULT_FLOOR.g_min && g <= DEFAULT_FLOOR.g_max &&
            b >= DEFAULT_FLOOR.b_min && b <= DEFAULT_FLOOR.b_max)
        {
            return 'X'; // unrecognized color
        }

        // Check for predefined colors
        for (const auto &color : COLORS)
        {
            if (r >= color.r_min && r <= color.r_max &&
                g >= color.g_min && g <= color.g_max &&
                b >= color.b_min && b <= color.b_max)
            {
                if (color.name == "Yellow")
                {
                    return 'Y';
                }
                else if (color.name == "Orange")
                {
                    return 'O';
                }
                else if (color.name == "Red")
                {
                    return 'R';
                }
                else
                {
                    return 'X';
                }
                // return "Detected Color: " + color.name + " (RGB: " + to_string(r) + ", " + to_string(g) + ", " + to_string(b) + ")";
            }
        }

        return 'X'; // Unrecognized color
    }

    int getDistanceSensors()
    {
        double distanceFront = getDistance(ds_front);
        double distanceLeft = getDistance(ds_left);
        double distanceRight = getDistance(ds_right);

        // cout << "Distance sensor values: Front=" << distanceFront << ", Left=" << distanceLeft << ", Right=" << distanceRight << endl;

        // Evaluate sensor readings and return corresponding identifier
        if (distanceFront > 0.39 && distanceLeft < 0.5 && distanceRight < 0.5)
        {
            // cout << "Wall Ahead" << endl;
            return 1;
        }
        else if (distanceFront < 0.39 && distanceLeft > 0.5 && distanceRight < 0.5)
        {
            // cout << "Wall Left" << endl;
            return 2;
        }
        else if (distanceFront < 0.39 && distanceLeft < 0.5 && distanceRight > 0.5)
        {
            // cout << "Wall Right" << endl;
            return 3;
        }
        else if (distanceFront > 0.39 && distanceLeft > 0.5 && distanceRight < 0.5)
        {
            // cout << "Wall Ahead and Left" << endl;
            return 4;
        }
        else if (distanceFront > 0.39 && distanceLeft < 0.5 && distanceRight > 0.5)
        {
            // cout << "Wall Ahead and Right" << endl;
            return 5;
        }
        else if (distanceFront < 0.39 && distanceLeft > 0.5 && distanceRight > 0.5)
        {
            // cout << "Wall Left and Right" << endl;
            return 6;
        }
        else if (distanceFront > 0.39 && distanceLeft > 0.5 && distanceRight > 0.5)
        {
            // cout << "Wall Ahead, Left and Right" << endl;
            return 7;
        }
        else
        {
            // cout << "No Walls Around" << endl;
            return 0; // No significant obstacles detected
        }
    }

    // Move the robot forward by a specified distance (in simulation steps)
    void goForward(int steps)
    {
        // cout << "Going forward for " << steps << " steps..." << endl;

        // PID controller parameters
        double Kp = 0.03;
        double Ki = 0.0;
        double Kd = 0.01;
        double previousError = 0.0;
        double integral = 0.0;
        if (steps == 0)
        {
            double initialLeftPosition = getLeftWheelSensor();

            // cout << "Initial Left Wheel Position: " << initialLeftPosition << endl;

            // Move forward until the left wheel rotates a specific distance
            while ((initialLeftPosition - getLeftWheelSensor()) < 30)
            {
                // cout << "Left Wheel Position difference: " << (initialLeftPosition - getLeftWheelSensor()) << endl;
                // Update global distance sensor readings midway
                if ((initialLeftPosition - getLeftWheelSensor()) >= 20.5 && (initialLeftPosition - getLeftWheelSensor()) < 21)
                {
                    wall_arrangement = getDistanceSensors();
                    // cout << "Wall Arrangement: " << wall_arrangement << endl;
                    floor_color = getFloorColor(floor_camera);
                    // cout << "Floor Color: " << floor_color << endl;
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
                double leftSpeed = 18 + correction;
                double rightSpeed = 18 - correction;

                leftMotor->setVelocity(-leftSpeed);
                rightMotor->setVelocity(-rightSpeed);
                step(timeStep); // Advance simulation
            }

            stopRobot(); // Stop robot after moving the required distance
        }
        else
        {
            for (int i = 0; i < steps; i++)
            {
                double initialLeftPosition = getLeftWheelSensor();

                // cout << "Initial Left Wheel Position: " << initialLeftPosition << endl;

                // Move forward until the left wheel rotates a specific distance
                while ((initialLeftPosition - getLeftWheelSensor()) < 20)
                {
                    // cout << "Left Wheel Position difference: " << (initialLeftPosition - getLeftWheelSensor()) << endl;
                    // Update global distance sensor readings midway
                    if ((initialLeftPosition - getLeftWheelSensor()) >= 10.5 && (initialLeftPosition - getLeftWheelSensor()) < 11)
                    {
                        wall_arrangement = getDistanceSensors();
                        // cout << "Wall Arrangement: " << wall_arrangement << endl;
                        floor_color = getFloorColor(floor_camera);
                        // cout << "Floor Color: " << floor_color << endl;
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
                    double leftSpeed = 18 + correction;
                    double rightSpeed = 18 - correction;

                    leftMotor->setVelocity(-leftSpeed);
                    rightMotor->setVelocity(-rightSpeed);
                    step(timeStep); // Advance simulation
                }

                stopRobot(); // Stop robot after moving the required distance
            }
        }
    }

    // Turn the robot left by 90 degrees using wheel encoders
    void turnLeft()
    {
        double initialLeftWheel = getLeftWheelSensor();
        double targetLeftWheel = initialLeftWheel + 2.1 * 3; // Approx. rotation for 90 degrees

        // Rotate until the left wheel reaches the target position
        while (getLeftWheelSensor() < targetLeftWheel)
        {
            leftMotor->setVelocity(2);
            rightMotor->setVelocity(-2);
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
        double targetRightWheel = initialRightWheel + 2.1 * 3; // Approx. rotation for 90 degrees

        // Rotate until the right wheel reaches the target position
        while (getRightWheelSensor() < targetRightWheel)
        {
            leftMotor->setVelocity(-2);
            rightMotor->setVelocity(2);
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

        while (distanceFront < 1.48 || distanceFront > 1.5)
        {
            // cout << distanceFront << endl;
            if (distanceFront < 1.48)
            {
                leftMotor->setVelocity(-1);
                rightMotor->setVelocity(-1);
            }
            else if (distanceFront > 1.5)
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

        while (fabs(distanceDifference) > 0.0009)
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
        const double leftX = 2.5;
        const double topY = -2;
        const double cellWidth = 0.25;
        const double cellHeight = 0.25;
        const int columns = 19;
        struct coordinate XY;

        int column = static_cast<int>((leftX - gps_x) / cellWidth);
        int row = static_cast<int>((gps_y - topY) / cellHeight);

        if (column < 0 || column > columns || row < 0 || row > columns)
        {
            cout << "Out of bounds" << endl;
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

    char toMoveForward(struct coordinate p, int wall_arrangement)
    {
        struct surroundCoor surCoor = getSurrounds(p);

        switch (orient)
        {
        case 0: // North
            if (isAccessible(wall_arrangement, p, surCoor.N) && visited[surCoor.N.y][surCoor.N.x] == false)
            {
                return 'F'; // Forward
            }
            else if (isAccessible(wall_arrangement, p, surCoor.W) && visited[surCoor.W.y][surCoor.W.x] == false)
            {
                return 'L'; // Left
            }
            else if (isAccessible(wall_arrangement, p, surCoor.E) && visited[surCoor.E.y][surCoor.E.x] == false)
            {
                return 'R'; // Right
            }
            else
            {
                return 'X'; // invalid move
            }
            break;
        case 1: // East
            if (isAccessible(wall_arrangement, p, surCoor.E) && visited[surCoor.E.y][surCoor.E.x] == false)
            {
                return 'F'; // Forward
            }
            else if (isAccessible(wall_arrangement, p, surCoor.N) && visited[surCoor.N.y][surCoor.N.x] == false)
            {
                return 'L'; // Left
            }
            else if (isAccessible(wall_arrangement, p, surCoor.S) && visited[surCoor.S.y][surCoor.S.x] == false)
            {
                return 'R'; // Right
            }
            else
            {
                return 'X'; // invalid move
            }
            break;
        case 2: // South
            if (isAccessible(wall_arrangement, p, surCoor.S) && visited[surCoor.S.y][surCoor.S.x] == false)
            {
                return 'F'; // Forward
            }
            else if (isAccessible(wall_arrangement, p, surCoor.E) && visited[surCoor.E.y][surCoor.E.x] == false)
            {
                return 'L'; // Left
            }
            else if (isAccessible(wall_arrangement, p, surCoor.W) && visited[surCoor.W.y][surCoor.W.x] == false)
            {
                return 'R'; // Right
            }
            else
            {
                return 'X'; // invalid move
            }
            break;
        case 3: // West
            if (isAccessible(wall_arrangement, p, surCoor.W) && visited[surCoor.W.y][surCoor.W.x] == false)
            {
                return 'F'; // Forward
            }
            else if (isAccessible(wall_arrangement, p, surCoor.S) && visited[surCoor.S.y][surCoor.S.x] == false)
            {
                return 'L'; // Left
            }
            else if (isAccessible(wall_arrangement, p, surCoor.N) && visited[surCoor.N.y][surCoor.N.x] == false)
            {
                return 'R'; // Right
            }
            else
            {
                return 'X'; // invalid move
            }
            break;
        }
        return 'X'; // invalid move
    }

    void moveRobot(char move, int robotcase)
    {
        switch (move)
        {
        case 'F':
            goForward(1);
            break;
        case 'L':
            if (wall_arrangement == 1 || wall_arrangement == 4 || wall_arrangement == 5 || wall_arrangement == 7)
            {
                if (!processVision(camera, display))
                {
                    // green_count++;
                    align_wall();
                    parallel_wall();
                }
                else
                {
                    // cout << "Green Detected" << endl;
                    if (robotcase == 0)
                    {
                        addToGreenCells(XY);
                    }
                }
            }
            turnLeft();
            goForward(1);
            break;
        case 'R':
            if (wall_arrangement == 1 || wall_arrangement == 4 || wall_arrangement == 5 || wall_arrangement == 7)
            {
                if (!processVision(camera, display))
                {
                    // green_count++;
                    align_wall();
                    parallel_wall();
                }
                else
                {
                    // cout << "Green Detected" << endl;
                    if (robotcase == 0)
                    {
                        addToGreenCells(XY);
                    }
                }
            }
            turnRight();
            goForward(1);
            break;
        case 'B':
            if (wall_arrangement == 1 || wall_arrangement == 4 || wall_arrangement == 5 || wall_arrangement == 7)
            {
                if (!processVision(camera, display))
                {
                    // green_count++;
                    align_wall();
                    parallel_wall();
                }
                else
                {
                    // cout << "Green Detected" << endl;
                    if (robotcase == 0)
                    {
                        addToGreenCells(XY);
                    }
                }
            }
            turnLeft();
            if (wall_arrangement == 1 || wall_arrangement == 4 || wall_arrangement == 5 || wall_arrangement == 7)
            {
                if (!processVision(camera, display))
                {
                    // green_count++;
                    // align_wall();
                    parallel_wall();
                }
                else
                {
                    // cout << "Green Detected" << endl;
                    if (robotcase == 0)
                    {
                        addToGreenCells(XY);
                    }
                }
            }
            turnLeft();
            goForward(1);
            break;
        default:
            break;
        }
    }

    bool isBranchable(struct coordinate p, int wall_arrangement)
    {
        struct surroundCoor surCoor = getSurrounds(p);
        int count = 0;

        if (isAccessible(wall_arrangement, p, surCoor.N) && visited[surCoor.N.y][surCoor.N.x] == false)
        {
            count++;
        }
        if (isAccessible(wall_arrangement, p, surCoor.W) && visited[surCoor.W.y][surCoor.W.x] == false)
        {
            count++;
        }
        if (isAccessible(wall_arrangement, p, surCoor.E) && visited[surCoor.E.y][surCoor.E.x] == false)
        {
            count++;
        }
        if (isAccessible(wall_arrangement, p, surCoor.S) && visited[surCoor.S.y][surCoor.S.x] == false)
        {
            count++;
        }

        if (count > 1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    char toMoveBackward(struct coordinate p, struct coordinate next_coor)
    {
        struct surroundCoor surCoor = getSurrounds(p);

        switch (orient)
        {
        case 0: // North
            if (compareCoordinates(next_coor, surCoor.N))
            {
                return 'F'; // Forward
            }
            else if (compareCoordinates(next_coor, surCoor.W))
            {
                return 'L'; // Left
            }
            else if (compareCoordinates(next_coor, surCoor.E))
            {
                return 'R'; // Right
            }
            else if (compareCoordinates(next_coor, surCoor.S))
            {
                return 'B'; // Backward
            }
            break;
        case 1: // East
            if (compareCoordinates(next_coor, surCoor.E))
            {
                return 'F'; // Forward
            }
            else if (compareCoordinates(next_coor, surCoor.N))
            {
                return 'L'; // Left
            }
            else if (compareCoordinates(next_coor, surCoor.S))
            {
                return 'R'; // Right
            }
            else if (compareCoordinates(next_coor, surCoor.W))
            {
                return 'B'; // Backward
            }
            break;
        case 2: // South
            if (compareCoordinates(next_coor, surCoor.S))
            {
                return 'F'; // Forward
            }
            else if (compareCoordinates(next_coor, surCoor.E))
            {
                return 'L'; // Left
            }
            else if (compareCoordinates(next_coor, surCoor.W))
            {
                return 'R'; // Right
            }
            else if (compareCoordinates(next_coor, surCoor.N))
            {
                return 'B'; // Backward
            }
            break;
        case 3: // West
            if (compareCoordinates(next_coor, surCoor.W))
            {
                return 'F'; // Forward
            }
            else if (compareCoordinates(next_coor, surCoor.S))
            {
                return 'L'; // Left
            }
            else if (compareCoordinates(next_coor, surCoor.N))
            {
                return 'R'; // Right
            }
            else if (compareCoordinates(next_coor, surCoor.E))
            {
                return 'B'; // Backward
            }
            break;

        default:
            break;
        }
        cout << "Invalid Move" << endl;
        return 'X';
    }

    bool isAroundVisited(struct coordinate p)
    {
        struct surroundCoor surCoor = getSurrounds(p);
        bool isTrue = false; // Declare isTrue

        switch (orient)
        {
        case 0: // North
            switch (wall_arrangement)
            {
            case 0:
                if (visited[surCoor.N.y][surCoor.N.x] && visited[surCoor.E.y][surCoor.E.x] && visited[surCoor.W.y][surCoor.W.x])
                    isTrue = true;
                break;
            case 1:
                if (visited[surCoor.E.y][surCoor.E.x] && visited[surCoor.W.y][surCoor.W.x])
                    isTrue = true;
                break;
            case 2:
                if (visited[surCoor.N.y][surCoor.N.x] && visited[surCoor.E.y][surCoor.E.x])
                    isTrue = true;
                break;
            case 3:
                if (visited[surCoor.N.y][surCoor.N.x] && visited[surCoor.W.y][surCoor.W.x])
                    isTrue = true;
                break;
            case 4:
                if (visited[surCoor.E.y][surCoor.E.x])
                    isTrue = true;
                break;
            case 5:
                if (visited[surCoor.W.y][surCoor.W.x])
                    isTrue = true;
                break;
            case 6:
                if (visited[surCoor.N.y][surCoor.N.x])
                    isTrue = true;
                break;
            }
            break;
        case 1: // East
            switch (wall_arrangement)
            {
            case 0:
                if (visited[surCoor.E.y][surCoor.E.x] && visited[surCoor.N.y][surCoor.N.x] && visited[surCoor.S.y][surCoor.S.x])
                    isTrue = true;
                break;
            case 1:
                if (visited[surCoor.S.y][surCoor.S.x] && visited[surCoor.N.y][surCoor.N.x])
                    isTrue = true;
                break;
            case 2:
                if (visited[surCoor.E.y][surCoor.E.x] && visited[surCoor.S.y][surCoor.S.x])
                    isTrue = true;
                break;
            case 3:
                if (visited[surCoor.E.y][surCoor.E.x] && visited[surCoor.N.y][surCoor.N.x])
                    isTrue = true;
                break;
            case 4:
                if (visited[surCoor.S.y][surCoor.S.x])
                    isTrue = true;
                break;
            case 5:
                if (visited[surCoor.N.y][surCoor.N.x])
                    isTrue = true;
                break;
            case 6:
                if (visited[surCoor.E.y][surCoor.E.x])
                    isTrue = true;
                break;
            }
            break;
        case 2: // South
            switch (wall_arrangement)
            {
            case 0:
                if (visited[surCoor.S.y][surCoor.S.x] && visited[surCoor.E.y][surCoor.E.x] && visited[surCoor.W.y][surCoor.W.x])
                    isTrue = true;
                break;
            case 1:
                if (visited[surCoor.W.y][surCoor.W.x] && visited[surCoor.E.y][surCoor.E.x])
                    isTrue = true;
                break;
            case 2:
                if (visited[surCoor.S.y][surCoor.S.x] && visited[surCoor.W.y][surCoor.W.x])
                    isTrue = true;
                break;
            case 3:
                if (visited[surCoor.S.y][surCoor.S.x] && visited[surCoor.E.y][surCoor.E.x])
                    isTrue = true;
                break;
            case 4:
                if (visited[surCoor.W.y][surCoor.W.x])
                    isTrue = true;
                break;
            case 5:
                if (visited[surCoor.E.y][surCoor.E.x])
                    isTrue = true;
                break;
            case 6:
                if (visited[surCoor.S.y][surCoor.S.x])
                    isTrue = true;
                break;
            }
            break;
        case 3: // West
            switch (wall_arrangement)
            {
            case 0:
                if (visited[surCoor.N.y][surCoor.N.x] && visited[surCoor.S.y][surCoor.S.x] && visited[surCoor.W.y][surCoor.W.x])
                    isTrue = true;
                break;
            case 1:
                if (visited[surCoor.N.y][surCoor.N.x] && visited[surCoor.S.y][surCoor.S.x])
                    isTrue = true;
                break;
            case 2:
                if (visited[surCoor.W.y][surCoor.W.x] && visited[surCoor.N.y][surCoor.N.x])
                    isTrue = true;
                break;
            case 3:
                if (visited[surCoor.W.y][surCoor.W.x] && visited[surCoor.S.y][surCoor.S.x])
                    isTrue = true;
                break;
            case 4:
                if (visited[surCoor.N.y][surCoor.N.x])
                    isTrue = true;
                break;
            case 5:
                if (visited[surCoor.S.y][surCoor.S.x])
                    isTrue = true;
                break;
            case 6:
                if (visited[surCoor.W.y][surCoor.W.x])
                    isTrue = true;
                break;
            }
            break;
        default:
            break;
        }
        return isTrue;
    }

    void addToGreenCells(struct coordinate p)
    {
        if (compareCoordinates(p, greenCells[0]) || compareCoordinates(p, greenCells[1]) || compareCoordinates(p, greenCells[2]))
        {
            cout << "Survivor Already Known" << endl;
            return;
        }
        else
        {
            if (greenptr < 3)
            {
                greenCells[greenptr] = p;
                cout << "Survivor Detected : " << p.x << "," << p.y << endl;
                greenptr++;
            }
            else
            {
                cout << "Survivor array is full" << endl;
            }
        }
    }

    void updateWalls(struct coordinate point)
    {
        if (wall_arrangement == 7)
        {
            if (orient == 0)
            {
                cells[point.y][point.x] = 13;
            } //|-|
            else if (orient == 1)
            {
                cells[point.y][point.x] = 12;
            } //_-|
            else if (orient == 2)
            {
                cells[point.y][point.x] = 11;
            } //|_|
            else if (orient == 3)
            {
                cells[point.y][point.x] = 14;
            } //|-_
        }
        else if (wall_arrangement == 6)
        {
            if (orient == 0)
            {
                cells[point.y][point.x] = 9;
            } //| |
            else if (orient == 1)
            {
                cells[point.y][point.x] = 10;
            } //_-
            else if (orient == 2)
            {
                cells[point.y][point.x] = 9;
            } //| |
            else if (orient == 3)
            {
                cells[point.y][point.x] = 10;
            } //_-
        }
        else if (wall_arrangement == 4)
        {
            if (orient == 0)
            {
                cells[point.y][point.x] = 8;
            } //|-
            else if (orient == 1)
            {
                cells[point.y][point.x] = 7;
            } //-|
            else if (orient == 2)
            {
                cells[point.y][point.x] = 6;
            } //_|
            else if (orient == 3)
            {
                cells[point.y][point.x] = 5;
            } //|_
        }
        else if (wall_arrangement == 5)
        {
            if (orient == 0)
            {
                cells[point.y][point.x] = 7;
            } //-|
            else if (orient == 1)
            {
                cells[point.y][point.x] = 6;
            } //_|
            else if (orient == 2)
            {
                cells[point.y][point.x] = 5;
            } //|_
            else if (orient == 3)
            {
                cells[point.y][point.x] = 8;
            } //|-
        }
        else if (wall_arrangement == 1)
        {
            if (orient == 0)
            {
                cells[point.y][point.x] = 2;
            } //-
            else if (orient == 1)
            {
                cells[point.y][point.x] = 3;
            } // |
            else if (orient == 2)
            {
                cells[point.y][point.x] = 4;
            } //_
            else if (orient == 3)
            {
                cells[point.y][point.x] = 1;
            } //|
        }
        else if (wall_arrangement == 2)
        {
            if (orient == 0)
            {
                cells[point.y][point.x] = 1;
            } //|
            else if (orient == 1)
            {
                cells[point.y][point.x] = 2;
            } //-
            else if (orient == 2)
            {
                cells[point.y][point.x] = 3;
            } // |
            else if (orient == 3)
            {
                cells[point.y][point.x] = 4;
            } //_
        }
        else if (wall_arrangement == 3)
        {
            if (orient == 0)
            {
                cells[point.y][point.x] = 3;
            } // |
            else if (orient == 1)
            {
                cells[point.y][point.x] = 4;
            } //_
            else if (orient == 2)
            {
                cells[point.y][point.x] = 1;
            } //|
            else if (orient == 3)
            {
                cells[point.y][point.x] = 2;
            } //-
        }
        else
        {
            cells[point.y][point.x] = 0; //
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// Functions for the floodfill ////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    bool isAccessibleFlood(struct coordinate p, struct coordinate p1)
    {
        if (p1.x < 0 || p1.y < 0 || p1.x >= ROWS || p1.y >= COLUMNS)
        {
            return false;
        }

        if (p.x == p1.x)
        {
            if (p.y > p1.y)
            {
                if ((cells[p.y][p.x] == 4 || cells[p.y][p.x] == 5 || cells[p.y][p.x] == 6 || cells[p.y][p.x] == 10 || cells[p.y][p.x] == 11 || cells[p.y][p.x] == 12 || cells[p.y][p.x] == 14) || (cell_colors[p1.y][p1.x] == 2 || cell_colors[p1.y][p1.x] == 3))
                {
                    return false;
                }

                else
                {
                    return true;
                }
            }
            else
            {
                if ((cells[p.y][p.x] == 2 || cells[p.y][p.x] == 7 || cells[p.y][p.x] == 8 || cells[p.y][p.x] == 10 || cells[p.y][p.x] == 12 || cells[p.y][p.x] == 13 || cells[p.y][p.x] == 14) || (cell_colors[p1.y][p1.x] == 2 || cell_colors[p1.y][p1.x] == 3))
                {
                    return false;
                }

                else
                {
                    return true;
                }
            }
        }
        else if (p.y == p1.y)
        {
            if (p.x > p1.x)
            {
                if ((cells[p.y][p.x] == 1 || cells[p.y][p.x] == 5 || cells[p.y][p.x] == 8 || cells[p.y][p.x] == 9 || cells[p.y][p.x] == 11 || cells[p.y][p.x] == 13 || cells[p.y][p.x] == 14) || (cell_colors[p1.y][p1.x] == 2 || cell_colors[p1.y][p1.x] == 3))
                {
                    return false;
                }
                else
                {
                    return true;
                }
            }
            else
            {
                if ((cells[p.y][p.x] == 3 || cells[p.y][p.x] == 6 || cells[p.y][p.x] == 7 || cells[p.y][p.x] == 9 || cells[p.y][p.x] == 11 || cells[p.y][p.x] == 12 || cells[p.y][p.x] == 13) || (cell_colors[p1.y][p1.x] == 2 || cell_colors[p1.y][p1.x] == 3))
                {
                    return false;
                }
                else
                {
                    return true;
                }
            }
        }
        else
        {
            return false;
        }
    }

    bool isConsistant(struct coordinate p)
    {
        struct surroundCoor surr = getSurrounds(p);
        int minVals[4] = {-1, -1, -1, -1};
        if (surr.N.x >= 0 && surr.N.y >= 0)
        {
            if (isAccessibleFlood(p, surr.N))
            {
                minVals[0] = flood[surr.N.y][surr.N.x];
            }
        }
        if (surr.E.x >= 0 && surr.E.y >= 0)
        {
            if (isAccessibleFlood(p, surr.E))
            {
                minVals[1] = flood[surr.E.y][surr.E.x];
            }
        }
        if (surr.S.x >= 0 && surr.S.y >= 0)
        {
            if (isAccessibleFlood(p, surr.S))
            {
                minVals[2] = flood[surr.S.y][surr.S.x];
            }
        }
        if (surr.W.x >= 0 && surr.W.y >= 0)
        {
            if (isAccessibleFlood(p, surr.W))
            {
                minVals[3] = flood[surr.W.y][surr.W.x];
            }
        }
        int val = flood[p.y][p.x];
        int minCount = 0;
        for (int i = 0; i < 4; i++)
        {
            if (minVals[i] == val - 1 && minVals[i] != -1)
            {
                minCount++;
            }
        }

        if (minCount > 0 || val == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void makeConsistant(struct coordinate p)
    {
        struct surroundCoor surr = getSurrounds(p);
        int minVals[4] = {-1, -1, -1, -1};
        if (surr.N.y >= 0 && surr.N.x >= 0)
        {
            if (isAccessibleFlood(p, surr.N))
            {
                minVals[0] = flood[surr.N.y][surr.N.x];
            }
        }
        if (surr.E.y >= 0 && surr.E.x >= 0)
        {
            if (isAccessibleFlood(p, surr.E))
            {
                minVals[1] = flood[surr.E.y][surr.E.x];
            }
        }
        if (surr.S.y >= 0 && surr.S.x >= 0)
        {
            if (isAccessibleFlood(p, surr.S))
            {
                minVals[2] = flood[surr.S.y][surr.S.x];
            }
        }
        if (surr.W.y >= 0 && surr.W.x >= 0)
        {
            if (isAccessibleFlood(p, surr.W))
            {
                minVals[3] = flood[surr.W.y][surr.W.x];
            }
        }
        int minimum = 1000;
        for (int i = 0; i < 4; i++)
        {
            if (minVals[i] == -1)
            {
                minVals[i] = 1000;
            }
            if (minVals[i] < minimum)
            {
                minimum = minVals[i];
            }
        }
        flood[p.y][p.x] = minimum + 1;
    }

    void floodFill(struct coordinate p, struct coordinate prev)
    {

        if (!isConsistant(p))
        {
            flood[p.y][p.x] = flood[prev.y][prev.x] + 1;
        }

        std::queue<coordinate> q;
        q.push(p);

        struct surroundCoor surr = getSurrounds(p);
        if (surr.N.x >= 0 && surr.N.y >= 0)
        {
            if (isAccessibleFlood(p, surr.N))
            {
                q.push(surr.N);
            }
        }
        if (surr.E.x >= 0 && surr.E.y >= 0)
        {
            if (isAccessibleFlood(p, surr.E))
            {
                q.push(surr.E);
            }
        }
        if (surr.S.x >= 0 && surr.S.y >= 0)
        {
            if (isAccessibleFlood(p, surr.S))
            {
                q.push(surr.S);
            }
        }
        if (surr.W.x >= 0 && surr.W.y >= 0)
        {
            if (isAccessibleFlood(p, surr.W))
            {
                q.push(surr.W);
            }
        }

        while (!q.empty())
        {
            struct coordinate crun = q.front();
            q.pop();
            if (isConsistant(crun))
            {
            }
            else
            {
                makeConsistant(crun);
                q.push(crun);
                struct surroundCoor surr = getSurrounds(crun);
                if (surr.N.y >= 0 && surr.N.x >= 0)
                {
                    if (isAccessibleFlood(crun, surr.N))
                    {
                        q.push(surr.N);
                    }
                }
                if (surr.E.y >= 0 && surr.E.x >= 0)
                {
                    if (isAccessibleFlood(crun, surr.E))
                    {
                        q.push(surr.E);
                    }
                }
                if (surr.S.y >= 0 && surr.S.x >= 0)
                {
                    if (isAccessibleFlood(crun, surr.S))
                    {
                        q.push(surr.S);
                    }
                }
                if (surr.W.y >= 0 && surr.W.x >= 0)
                {
                    if (isAccessibleFlood(crun, surr.W))
                    {
                        q.push(surr.W);
                    }
                }
            }
        }
    }

    char toMove(struct coordinate p, struct coordinate prevPos, int orient)
    {
        struct surroundCoor surr = getSurrounds(p);

        //	int val = flood[p.x][p.y];
        int minVals[4] = {1000, 1000, 1000, 1000};
        int prevDir = 0;
        if (surr.N.y >= 0 && surr.N.x >= 0)
        {
            if (isAccessibleFlood(p, surr.N))
            {
                minVals[0] = flood[surr.N.y][surr.N.x];
                if (surr.N.x == prevPos.x && surr.N.y == prevPos.y)
                {
                    prevDir = 0;
                }
            }
        }
        if (surr.E.y >= 0 && surr.E.x >= 0)
        {
            if (isAccessibleFlood(p, surr.E))
            {
                minVals[1] = flood[surr.E.y][surr.E.x];
                if (surr.E.x == prevPos.x && surr.E.y == prevPos.y)
                {
                    prevDir = 1;
                }
            }
        }
        if (surr.S.y >= 0 && surr.S.x >= 0)
        {
            if (isAccessibleFlood(p, surr.S))
            {
                minVals[2] = flood[surr.S.y][surr.S.x];
                if (surr.S.x == prevPos.x && surr.S.y == prevPos.y)
                {
                    prevDir = 2;
                }
            }
        }
        if (surr.W.y >= 0 && surr.W.x >= 0)
        {
            if (isAccessibleFlood(p, surr.W))
            {
                minVals[3] = flood[surr.W.y][surr.W.x];
                if (surr.W.x == prevPos.x && surr.W.y == prevPos.y)
                {
                    prevDir = 3;
                }
            }
        }

        int minimum = 1000;
        int noMovements = 0;
        for (int i = 0; i < 4; i++)
        {
            if (minVals[i] != 1000)
            {
                noMovements++;
            }
        }

        int minCell = 0;
        for (int i = 0; i < 4; i++)
        {
            if (minVals[i] < minimum)
            {
                if (noMovements == 1)
                {
                    minimum = minVals[i];
                    minCell = i;
                }
                else
                {
                    if (i == prevDir)
                    {
                    }
                    else
                    {
                        minimum = minVals[i];
                        minCell = i;
                    }
                }
            }
        }

        using namespace std;
        string line = "";
        for (int b = 0; b < 4; b++)
        {
            line = line + to_string(minVals[b]) + "\t";
        }
        string minCellS = to_string(minCell);

        if (minCell == orient)
        {
            return 'F';
        }
        else if (minCell == orient - 1 || minCell == orient + 3)
        {
            return 'L';
        }
        else if (minCell == orient + 1 || minCell == orient - 3)
        {
            return 'R';
        }
        else
        {
            return 'B';
        }
    }

    void initializeFlood(int dest_x, int dest_y)
    {
        for (int i = 0; i < ROWS; i++)
        {
            for (int j = 0; j < COLUMNS; j++)
            {
                flood[i][j] = 1000; // Set a high value for all cells
            }
        }
        flood[dest_y][dest_x] = 0; // Set the new destination
    }

    void updateCoordinates()
    {
        // Log robot's position and cell
        const double *gpsValues = gps->getValues();
        double gps_x = gpsValues[0];
        double gps_y = gpsValues[1];

        XY_prev = XY;
        XY = calculateCell(gps_x, gps_y);
    }

    // void buildFirepitWalls() {
    //     for (int i = 0; i < ROWS; i++) {
    //         for (int j = 0; j < COLUMNS; j++) {
    //             if (cell_colors[i][j] > 0) { // Fire pit cell
    //                 // Check adjacent cells to determine wall placement
    //                 bool left = (j == 0 || cell_colors[i][j - 1] == -1);
    //                 bool right = (j == COLUMNS - 1 || cell_colors[i][j + 1] == -1);
    //                 bool top = (i == 0 || cell_colors[i - 1][j] == -1);
    //                 bool bottom = (i == ROWS - 1 || cell_colors[i + 1][j] == -1);

    //                 // Assign appropriate wall number
    //                 if (left && bottom) cells[i][j] = 5;
    //                 else if (right && bottom) cells[i][j] = 6;
    //                 else if (right && top) cells[i][j] = 7;
    //                 else if (left && top) cells[i][j] = 8;
    //                 else if (left) cells[i][j] = 1;
    //                 else if (top) cells[i][j] = 2;
    //                 else if (right) cells[i][j] = 3;
    //                 else if (bottom) cells[i][j] = 4;
    //             }
    //         }
    //     }
    // }

    void identifyFirepits(struct coordinate p)
    {
        switch (floor_color)
        {
        case 'R':
            cell_colors[p.y][p.x] = 3;
            break;
        case 'O':
            cell_colors[p.y][p.x] = 2;
            break;
        case 'Y':
            cell_colors[p.y][p.x] = 1;
            break;
        default:
            break;
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////

    // Main simulation loop
    void run()
    {

        cout << "Starting simulation loop..." << endl;

        // Initial delay (3 seconds)
        for (int i = 0; i < 1000 / timeStep; ++i)
        {
            step(timeStep);
        }

        // Get initial distance sensor readings
        // wall_arrangement = getDistanceSensors();
        int dfs_case = 0;
        int robot_case = 0;
        int floodcase = 0;

        cout << "Exploring the maze and Finding Survivros..." << endl;

        goForward(0); // This is to enter the maze
        align_wall();
        parallel_wall();
        // important ###########################################################################
        //  This is not for the correct starting position
        //  Have to create a function to enter the maze
        // #####################################################################################

        while (step(timeStep) != -1)
        {
            // Print front distance sensor value
            // double distanceFront = getDistance(ds_front);
            // cout << "Distance sensor values: Front=" << distanceFront << endl;

            updateCoordinates();

            int x = XY.x;
            int y = XY.y;

            // if (x != -1 && y != -1)
            // {
            //     cout << "GPS Coordinates: X=" << gps_x << ", Y=" << gps_y << " | X =" << x << ", Y =" << y << endl;
            // }

            switch (robot_case)
            {
            case 0: // To explore the maze
                visited[y][x] = true;
                updateWalls(XY);
                identifyFirepits(XY);

                // Update wall arrangement in cells
                switch (dfs_case)
                {
                case 0: // Going until a dead end is found
                    if (wall_arrangement == 7 || isAroundVisited(XY))
                    { // Dead end
                        // cout << "Dead End" << endl;
                        dfs_case = 1;
                    }
                    else
                    {
                        location_stack->push(XY);
                        if (isBranchable(XY, wall_arrangement))
                        {
                            branch_stack->push(XY);
                        }

                        char move = toMoveForward(XY, wall_arrangement);
                        moveRobot(move, robot_case);
                    }
                    break;

                case 1: // Going backward until a branch is found
                    if (compareCoordinates(XY, branch_stack->top()))
                    {
                        dfs_case = 0;
                        branch_stack->pop();
                    }
                    else
                    {
                        struct coordinate next_coor = location_stack->top();
                        location_stack->pop();

                        char move = toMoveBackward(XY, next_coor);
                        moveRobot(move, robot_case);

                        if (location_stack->empty())
                        {
                            cout << "Maze exploration finished" << endl;
                            robot_case = 1;
                        }
                    }
                    break;

                default:
                    break;
                }
                break;

            case 1: // go out of the maze and come to the starting position
                    // ##################################
                //  Again this is not the currect starting position
                switch (orient)
                {
                case 0: // North
                    moveRobot('B', robot_case);
                    turnLeft();
                    turnLeft();
                    break;
                case 1: // East
                    moveRobot('R', robot_case);
                    turnLeft();
                    turnLeft();
                    break;
                case 2: // South
                    moveRobot('F', robot_case);
                    turnLeft();
                    turnLeft();
                    break;
                case 3: // West
                    moveRobot('L', robot_case);
                    turnLeft();
                    turnLeft();
                    break;
                default:
                    break;
                }

                cout << "Robot Came to the Starting Position" << endl;
                robot_case = 2;

                // wait for 3 seconds
                for (int i = 0; i < 3000 / timeStep; ++i)
                {
                    step(timeStep);
                }

                break;

            case 2: // Rescue the Survivors

                switch (floodcase)
                {
                case 0:

                    cout << "Starting the Rescue Mission..." << endl;
                    goForward(1); // This is to enter the maze
                    // This has to be fixed to the correct starting position
                    struct coordinate survCoor;
                    // cout << "Building FirePit Walls" << endl;
                    // buildFirepitWalls();
                    // cout << "Firepit Walls Built" << endl;
                    greenptr = 0;
                    floodcase = 1;
                    break;
                case 1: // Initialize the flood fill
                {
                    survCoor = greenCells[greenptr];

                    int survivor_x = survCoor.x;
                    int survivor_y = survCoor.y;
                    cout << "Next Survivor Coordinates: X=" << survivor_x << ", Y=" << survivor_y << endl;

                    initializeFlood(survivor_x, survivor_y);

                    // cout << "Flood Fill Started" << endl;

                    floodFill(survCoor, survCoor);

                    // cout << "Flood Fill Finished" << endl;

                    floodcase = 2;
                    break;
                }

                case 2: // Move to the survivor
                {
                    if (flood[XY.y][XY.x] == 0)
                    {
                        cout << "Survivor " << (greenptr + 1) << " Found : Starting Rescue Process" << endl;
                        greenptr++;
                        floodcase = 1;

                        cout << "Rescue Process Ongoing : Waiting for 3 seconds" << endl;

                        // wait for 3 seconds
                        for (int i = 0; i < 3000 / timeStep; ++i)
                        {
                            step(timeStep);
                        }

                        cout << "Survivor " << (greenptr) << " Rescued" << endl;

                        if (greenptr == 3)
                        {
                            cout << "All Survivors Rescued !!!" << endl;
                            floodcase = 3;
                        }
                    }
                    else
                    {
                        char move = toMove(XY, XY_prev, orient);
                        moveRobot(move, robot_case);
                    }
                    break;
                }

                case 3: // Set floodfill to the starting position
                    initializeFlood(10, 0);

                    // cout << "Flood Fill Started" << endl;

                    struct coordinate startCoor;
                    startCoor.x = 10;
                    startCoor.y = 0;

                    floodFill(startCoor, startCoor);

                    // cout << "Flood Fill Finished" << endl;
                    floodcase = 4;
                    break;

                case 4: // Move to the starting position
                    if (flood[XY.y][XY.x] == 0)
                    {
                        // cout << "Reached the Starting Position" << endl;
                        floodcase = 5;
                        robot_case = 1;
                    }
                    else
                    {
                        char move = toMove(XY, XY_prev, orient);
                        moveRobot(move, robot_case);
                    }
                    break;

                default:
                    break;
                }

                break;

            default:
                break;
            }
        }
    }

private:
    int timeStep;
    int wall_arrangement;
    char floor_color;
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
    Camera *camera;
    Camera *floor_camera;
    Display *display;
};

// Entry point of the program
int main()
{
    MyRobot robot; // Create an instance of the robot
    // robot.moveFirstCell();
    robot.run(); // Start the simulation loop
    return 0;
}