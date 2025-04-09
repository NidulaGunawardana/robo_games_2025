#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <iostream>
#include <string>

#define TIME_STEP 32  // Webots simulation step
#define MAX_SPEED 6.28  // Maximum motor speed
#define GREEN_THRESHOLD 70  // Minimum green intensity for detection
#define GREEN_AREA_THRESHOLD 0.08  // 8% of the image must be green to trigger detection

using namespace webots;
using namespace std;

/**
 * Function to process the camera image, detect green areas, and display the binary mask.
 * @param camera - Pointer to the Webots camera.
 * @param display - Pointer to the Webots display.
 * @return true if a large green area is detected, false otherwise.
 */
bool processVision(Camera *camera, Display *display) {
    if (!camera || !display) {
        cerr << "ERROR: Camera or Display not initialized!" << endl;
        return false;
    }

    int width = camera->getWidth();
    int height = camera->getHeight();

    // Create an empty image buffer
    unsigned char *imageBuffer = new unsigned char[3 * width * height];

    // Get the camera image
    const unsigned char *image = camera->getImage();
    if (!image) {
        cerr << "ERROR: Failed to get camera image!" << endl;
        delete[] imageBuffer;
        return false;
    }

    int green_count = 0;
    int total_pixels = width * height;

    // Process Image: Detect Green & Create Binary Mask
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            int index = (y * width + x) * 4;  // BGRA format (4 bytes per pixel)

            int r = image[index + 2];  // Red
            int g = image[index + 1];  // Green
            int b = image[index];      // Blue

            // Check if green is dominant
            bool isGreen = (g > GREEN_THRESHOLD && g > r && g > b);
            if (isGreen) green_count++;

            // Create binary mask (Green -> Green, Others -> Black)
            imageBuffer[3 * (y * width + x)] = isGreen ? 0 : 0;        // Red
            imageBuffer[3 * (y * width + x) + 1] = isGreen ? 255 : 0;  // Green
            imageBuffer[3 * (y * width + x) + 2] = isGreen ? 0 : 0;    // Blue
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
std::string getFloorRGB(Camera *camera) {
    if (!camera) return "ERROR: Floor Camera Not Found";

    const unsigned char *image = camera->getImage();
    if (!image) return "ERROR: Failed to get floor image";

    int width = camera->getWidth();
    int height = camera->getHeight();

    // Take the center pixel as the reference
    int x = width / 2;
    int y = height / 2;
    int index = (y * width + x) * 4;  // BGRA format

    int r = image[index + 2];  // Red
    int g = image[index + 1];  // Green
    int b = image[index];      // Blue

    // Format the RGB values into a string
    return "RGB: (" + std::to_string(r) + ", " + std::to_string(g) + ", " + std::to_string(b) + ")";
}


int main() {
    // Initialize Webots robot
    Robot *robot = new Robot();

    // Get motors
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

    // Initialize cameras
    Camera *camera = robot->getCamera("camera");  // Main camera
    Camera *floor_camera = robot->getCamera("camera_2");  // Floor camera

    if (!camera || !floor_camera) {
        cerr << "ERROR: One or both cameras not found!" << endl;
        return -1;
    }
    camera->enable(TIME_STEP);
    floor_camera->enable(TIME_STEP);

    // Initialize display
    Display *display = robot->getDisplay("display");
    if (!display) {
        cerr << "WARNING: Display device not found. Camera will work, but no image will be displayed." << endl;
        return -1;
    }

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
                break;
            case Keyboard::DOWN:
                left_speed = -MAX_SPEED;
                right_speed = -MAX_SPEED;
                break;
            case Keyboard::LEFT:
                left_speed = -MAX_SPEED / 2;
                right_speed = MAX_SPEED / 2;
                break;
            case Keyboard::RIGHT:
                left_speed = MAX_SPEED / 2;
                right_speed = -MAX_SPEED / 2;
                break;
            case Keyboard::END:  // ESC key
                cout << "Exiting..." << endl;
                delete robot;
                return 0;
            default:
                break;
        }

        // Set motor speeds
        left_motor->setVelocity(left_speed);
        right_motor->setVelocity(right_speed);

        // Process vision from main camera
        bool greenDetected = processVision(camera, display);
        if (greenDetected) {
            cout << "Green Area Detected!" << endl;
        } else {
            cout << "No Green Detected." << endl;
        }

        // Detect floor color using the second camera
        string floorColor = getFloorRGB(floor_camera);
        cout << "Floor Color Detected: " << floorColor << endl;
    }

    // Cleanup
    delete robot;
    return 0;
}
