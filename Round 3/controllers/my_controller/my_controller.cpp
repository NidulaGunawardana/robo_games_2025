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

#define GREEN_THRESHOLD 70        // Minimum green intensity for detection
#define GREEN_AREA_THRESHOLD 0.08 // 8% of the image must be green to trigger detection

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

const int ROWS = 20;
const int COLUMNS = 20;

bool visited[ROWS][COLUMNS] = {false};

// std::stack<struct coordinate> location_stack;
// std::stack<struct coordinate> branch_stack;

auto location_stack = std::make_unique<std::stack<struct coordinate, std::vector<struct coordinate>>>();
auto branch_stack = std::make_unique<std::stack<struct coordinate, std::vector<struct coordinate>>>();

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
                bool isGreen = (g > GREEN_THRESHOLD && g > r && g > b);
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
     * Function to detect the dominant floor color (Yellow, Orange, or Red).
     * @param camera - Pointer to the Webots floor camera.
     * @return string representing the detected floor color.
     */
    std::string getFloorRGB(Camera *camera)
    {
        if (!camera)
            return "ERROR: Floor Camera Not Found";

        const unsigned char *image = camera->getImage();
        if (!image)
            return "ERROR: Failed to get floor image";

        int width = camera->getWidth();
        int height = camera->getHeight();

        // Take the center pixel as the reference
        int x = width / 2;
        int y = height / 2;
        int index = (y * width + x) * 4; // BGRA format

        int r = image[index + 2]; // Red
        int g = image[index + 1]; // Green
        int b = image[index];     // Blue

        // Format the RGB values into a string
        return "RGB: (" + std::to_string(r) + ", " + std::to_string(g) + ", " + std::to_string(b) + ")";
    }

    int getDistanceSensors()
    {
        double distanceFront = getDistance(ds_front);
        double distanceLeft = getDistance(ds_left);
        double distanceRight = getDistance(ds_right);

        cout << "Distance sensor values: Front=" << distanceFront << ", Left=" << distanceLeft << ", Right=" << distanceRight << endl;

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

            // cout << "Initial Left Wheel Position: " << initialLeftPosition << endl;

            // Move forward until the left wheel rotates a specific distance
            while ((initialLeftPosition - getLeftWheelSensor()) < 20)
            {
                // cout << "Left Wheel Position difference: " << (initialLeftPosition - getLeftWheelSensor()) << endl;
                // Update global distance sensor readings midway
                if ((initialLeftPosition - getLeftWheelSensor()) >= 10.5 && (initialLeftPosition - getLeftWheelSensor()) < 11)
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
        double targetLeftWheel = initialLeftWheel + 2.1 * 3; // Approx. rotation for 90 degrees

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
        double targetRightWheel = initialRightWheel + 2.1 * 3; // Approx. rotation for 90 degrees

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

        while (fabs(distanceDifference) > 0.0005)
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

    // int current_cell(struct coordinate p)
    // { // Returns the flood cell number of the current cell
    //     int cell_no = 0;
    //     if (p.x >= 0 && p.x < ROWS && p.y >= 0 && p.y < COLUMNS)
    //     {
    //         cell_no = flood[p.y][p.x];
    //     }
    //     else
    //     {
    //         p.x = -1;
    //         p.y = -1;
    //         // Handle out-of-bounds case
    //         // throw an error or assign a default value
    //         cout << "Flood Out of bounds" << endl;
    //     }

    //     return cell_no;
    // }

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

    // char toMove(struct coordinate p, int wall_arrangement)
    // {
    //     struct surroundCoor surCoor = getSurrounds(p);
    //     // int min = current_cell(p);
    //     // cout << "Current cell no: " << min << endl;
    //     struct coordinate next_coor = p;

    //     int directionsLooked = 0;

    //     while (true)
    //     {
    //         bool exitLoop = false; // Flag to exit the while loop

    //         switch (directionsLooked)
    //         {
    //         case 0:
    //             if ((surCoor.N.x < 10 && surCoor.N.y < 10) && (surCoor.N.x >= 0 && surCoor.N.y >= 0))
    //             {
    //                 // int cell_no = current_cell(surCoor.N);
    //                 cout << "Looking N: " << cell_no << endl;
    //                 if ((cell_no == min - 1) && isAccessible(wall_arrangement, p, surCoor.N))
    //                 {
    //                     min = cell_no;
    //                     next_coor = surCoor.N;
    //                     exitLoop = true; // Set the flag to exit the while loop
    //                     break;           // Break the switch
    //                 }
    //             }
    //             directionsLooked++;
    //             break;

    //         case 1:
    //             if ((surCoor.W.x < 10 && surCoor.W.y < 10) && (surCoor.W.x >= 0 && surCoor.W.y >= 0))
    //             {
    //                 // int cell_no = current_cell(surCoor.W);
    //                 cout << "Looking W: " << cell_no << endl;
    //                 if ((cell_no == min - 1) && isAccessible(wall_arrangement, p, surCoor.W))
    //                 {
    //                     min = cell_no;
    //                     next_coor = surCoor.W;
    //                     exitLoop = true; // Set the flag to exit the while loop
    //                     break;           // Break the switch
    //                 }
    //             }
    //             directionsLooked++;
    //             break;

    //         case 3:
    //             if ((surCoor.S.x < 10 && surCoor.S.y < 10) && (surCoor.S.x >= 0 && surCoor.S.y >= 0))
    //             {
    //                 // int cell_no = current_cell(surCoor.S);
    //                 cout << "Looking S: " << cell_no << endl;
    //                 if ((cell_no == min - 1) && isAccessible(wall_arrangement, p, surCoor.S))
    //                 {
    //                     min = cell_no;
    //                     next_coor = surCoor.S;
    //                     exitLoop = true; // Set the flag to exit the while loop
    //                     break;           // Break the switch
    //                 }
    //             }
    //             directionsLooked++;
    //             break;

    //         case 2:
    //             if ((surCoor.E.x < 10 && surCoor.E.y < 10) && (surCoor.E.x >= 0 && surCoor.E.y >= 0))
    //             {
    //                 // int cell_no = current_cell(surCoor.E);
    //                 cout << "Looking E: " << cell_no << endl;
    //                 if ((cell_no == min - 1) && isAccessible(wall_arrangement, p, surCoor.E))
    //                 {
    //                     min = cell_no;
    //                     next_coor = surCoor.E;
    //                     exitLoop = true; // Set the flag to exit the while loop
    //                     break;           // Break the switch
    //                 }
    //             }
    //             directionsLooked++;
    //             break;

    //         default:
    //             break;
    //         }

    //         if (exitLoop)
    //         { // Exit the while loop if the flag is set
    //             break;
    //         }
    //     }

    //     switch (orient)
    //     {
    //     case 0:
    //         if (compareCoordinates(next_coor, surCoor.N))
    //         {
    //             return 'F'; // Forward
    //         }
    //         else if (compareCoordinates(next_coor, surCoor.W))
    //         {
    //             return 'L'; // Left
    //         }
    //         else if (compareCoordinates(next_coor, surCoor.S))
    //         {
    //             return 'B'; // Backward
    //         }
    //         else if (compareCoordinates(next_coor, surCoor.E))
    //         {
    //             return 'R'; // Right
    //         }
    //         break;

    //     case 1:
    //         if (compareCoordinates(next_coor, surCoor.E))
    //         {
    //             return 'F'; // Forward
    //         }
    //         else if (compareCoordinates(next_coor, surCoor.N))
    //         {
    //             return 'L'; // Left
    //         }
    //         else if (compareCoordinates(next_coor, surCoor.W))
    //         {
    //             return 'B'; // Backward
    //         }
    //         else if (compareCoordinates(next_coor, surCoor.S))
    //         {
    //             return 'R'; // Right
    //         }
    //         break;

    //     case 2:
    //         if (compareCoordinates(next_coor, surCoor.S))
    //         {
    //             return 'F'; // Forward
    //         }
    //         else if (compareCoordinates(next_coor, surCoor.E))
    //         {
    //             return 'L'; // Left
    //         }
    //         else if (compareCoordinates(next_coor, surCoor.N))
    //         {
    //             return 'B'; // Backward
    //         }
    //         else if (compareCoordinates(next_coor, surCoor.W))
    //         {
    //             return 'R'; // Right
    //         }
    //         break;

    //     case 3:
    //         if (compareCoordinates(next_coor, surCoor.W))
    //         {
    //             return 'F'; // Forward
    //         }
    //         else if (compareCoordinates(next_coor, surCoor.S))
    //         {
    //             return 'L'; // Left
    //         }
    //         else if (compareCoordinates(next_coor, surCoor.E))
    //         {
    //             return 'B'; // Backward
    //         }
    //         else if (compareCoordinates(next_coor, surCoor.N))
    //         {
    //             return 'R'; // Right
    //         }
    //         break;

    //     default:
    //         break;
    //     }

    //     return 'X'; // No valid move
    // }

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

    void moveRobot(char move)
    {
        switch (move)
        {
        case 'F':
            goForward(1);
            break;
        case 'L':
            if (wall_arrangement == 1 || wall_arrangement == 4 || wall_arrangement == 5)
            {
                if (!processVision(camera, display))
                {
                    // green_count++;
                    align_wall();
                    parallel_wall();
                }
                else
                {
                    cout << "Green Detected" << endl;
                }
            }
            turnLeft();
            goForward(1);
            break;
        case 'R':
            if (wall_arrangement == 1 || wall_arrangement == 4 || wall_arrangement == 5)
            {
                if (!processVision(camera, display))
                {
                    // green_count++;
                    align_wall();
                    parallel_wall();
                }
                else
                {
                    cout << "Green Detected" << endl;
                }
            }
            turnRight();
            goForward(1);
            break;
        case 'B':
            if (wall_arrangement == 1 || wall_arrangement == 4 || wall_arrangement == 5)
            {
                if (!processVision(camera, display))
                {
                    // green_count++;
                    align_wall();
                    parallel_wall();
                }
                else
                {
                    cout << "Green Detected" << endl;
                }
            }
            turnLeft();
            if (wall_arrangement == 1 || wall_arrangement == 4 || wall_arrangement == 5)
            {
                if (!processVision(camera, display))
                {
                    // green_count++;
                    align_wall();
                    parallel_wall();
                }
                else
                {
                    cout << "Green Detected" << endl;
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

    bool isAroundVisited(struct coordinate p) {
        struct surroundCoor surCoor = getSurrounds(p);
        bool isTrue = false;  // Declare isTrue
    
        switch (orient) {
            case 0: // North
                switch (wall_arrangement) {
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
                switch (wall_arrangement) {
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
                switch (wall_arrangement) {
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
                switch (wall_arrangement) {
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
        int robot_case = 0;

        goForward(1);

        while (step(timeStep) != -1)
        {
            // Print front distance sensor value
            // double distanceFront = getDistance(ds_front);
            // cout << "Distance sensor values: Front=" << distanceFront << endl;

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

            visited[y][x] = true;

            // update wall arrangement in cells

            switch (robot_case)
            {
            case 0: // Going until a dead end is found
                /* code */
                if (wall_arrangement == 7 || isAroundVisited(XY)) // Dead end
                {
                    robot_case = 1;
                }
                else
                {
                    location_stack->push(XY);
                    if (isBranchable(XY, wall_arrangement))
                    {
                        branch_stack->push(XY);
                    }

                    char move = toMoveForward(XY, wall_arrangement);
                    moveRobot(move);
                }
                break;

            case 1: // Going backward until a branch is found
                if (compareCoordinates(XY, branch_stack->top()))
                {
                    robot_case = 0;
                    branch_stack->pop();
                }
                else
                {
                    struct coordinate next_coor = location_stack->top();
                    location_stack->pop();
                    char move = toMoveBackward(XY, next_coor);
                    moveRobot(move);
                }

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