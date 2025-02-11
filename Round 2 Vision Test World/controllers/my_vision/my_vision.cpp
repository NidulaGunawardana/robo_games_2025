#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <iostream>
#include <cstring>  // Needed for memcpy

#define TIME_STEP 32  // Webots simulation step
#define MAX_SPEED 6.28  // Maximum motor speed
#define GREEN_THRESHOLD 70  // Minimum green intensity for detection
#define GREEN_AREA_THRESHOLD 0.08  // 20% of the image must be green to trigger detection

using namespace webots;
using namespace std;

/**
 * Function to detect a significant green area.
 * @param image - Pointer to the camera image data.
 * @param width - Image width.
 * @param height - Image height.
 * @return true if a large green area is detected, false otherwise.
 */
bool detectGreenBlob(const unsigned char *image, int width, int height) {
    int green_count = 0;
    int total_pixels = width * height;

    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            int index = (y * width + x) * 4;  // BGRA format (4 bytes per pixel)
            
            int r = image[index + 2];  // Red
            int g = image[index + 1];  // Green
            int b = image[index];      // Blue

            // Count pixels where green is dominant and above threshold
            if (g > GREEN_THRESHOLD && g > r && g > b) {
                green_count++;
            }
        }
    }

    // Check if green area is larger than the threshold percentage
    if (static_cast<double>(green_count) / total_pixels > GREEN_AREA_THRESHOLD) {
        cout << "Green Area Detected!" << endl;
        return true;
    }
    cout << "No Green" << endl;
    return false;
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

    // Initialize camera
    Camera *camera = robot->getCamera("camera");
    if (!camera) {
        cerr << "ERROR: Camera not found!" << endl;
        return -1;
    }
    camera->enable(TIME_STEP);
    int width = camera->getWidth();
    int height = camera->getHeight();

    // Initialize display
    Display *display = robot->getDisplay("display");
    if (!display) {
        cerr << "WARNING: Display device not found. Camera will work, but no image will be displayed." << endl;
        return -1;
    }

    // Create an empty image buffer
    unsigned char *imageBuffer = new unsigned char[3 * width * height];

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
                delete[] imageBuffer;
                delete robot;
                return 0;
            default:
                break;
        }

        // Set motor speeds
        left_motor->setVelocity(left_speed);
        right_motor->setVelocity(right_speed);

        // Process Camera Image
        const unsigned char *image = camera->getImage();
        if (image) {
            // **1️⃣ Detect Green Area**
            bool greenDetected = detectGreenBlob(image, width, height);

            // **2️⃣ Extract Green Color (Binary Mask)**
            for (int x = 0; x < width; x++) {
                for (int y = 0; y < height; y++) {
                    int index = (y * width + x) * 4;  // BGRA format (4 bytes per pixel)
                    
                    int r = image[index + 2];  // Red
                    int g = image[index + 1];  // Green
                    int b = image[index];      // Blue

                    // Thresholding: Keep only green pixels
                    if (g > GREEN_THRESHOLD && g > r && g > b) {
                        imageBuffer[3 * (y * width + x)] = 0;      // Red (0)
                        imageBuffer[3 * (y * width + x) + 1] = 255;  // Green (255)
                        imageBuffer[3 * (y * width + x) + 2] = 0;  // Blue (0)
                    } else {
                        imageBuffer[3 * (y * width + x)] = 0;      // Black background
                        imageBuffer[3 * (y * width + x) + 1] = 0;
                        imageBuffer[3 * (y * width + x) + 2] = 0;
                    }
                }
            }

            // **3️⃣ Display the Binary Green Mask**
            ImageRef *image_ref = display->imageNew(width, height, imageBuffer, Display::RGB);
            display->imagePaste(image_ref, 0, 0, false);
            display->imageDelete(image_ref);
        }
    }

    // Cleanup
    delete[] imageBuffer;
    delete robot;
    return 0;
}
