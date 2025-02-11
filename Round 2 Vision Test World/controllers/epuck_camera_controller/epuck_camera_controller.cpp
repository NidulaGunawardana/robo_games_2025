#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/Keyboard.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#define TIME_STEP 32  // Webots simulation step (ms)

using namespace webots;
using namespace cv;

int main() {
    // Initialize the Webots robot
    Robot *robot = new Robot();

    // Get the camera
    Camera *camera = robot->getCamera("camera");
    camera->enable(TIME_STEP);

    int width = camera->getWidth();
    int height = camera->getHeight();

    std::cout << "Camera Initialized: " << width << "x" << height << std::endl;

    while (robot->step(TIME_STEP) != -1) {
        // Get the camera image
        const unsigned char *image = camera->getImage();

        if (image) {
            // Convert Webots camera image to OpenCV format
            Mat frame(height, width, CV_8UC4, (void *)image);

            // Convert RGBA to BGR for OpenCV processing
            Mat frameBGR;
            cvtColor(frame, frameBGR, COLOR_RGBA2BGR);

            // Apply simple image processing (e.g., convert to grayscale)
            Mat grayFrame;
            cvtColor(frameBGR, grayFrame, COLOR_BGR2GRAY);

            // Display the processed image
            imshow("e-Puck Camera - Grayscale", grayFrame);
            if (waitKey(1) == 27) break; // Exit on ESC key
        }
    }

    delete robot;
    return 0;
}
