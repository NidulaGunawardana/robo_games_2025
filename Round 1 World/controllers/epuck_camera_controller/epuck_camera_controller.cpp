#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#define TIME_STEP 32 // Webots simulation time step (ms)

using namespace webots;
using namespace cv;

int main() {
    // Initialize the Webots Robot
    Robot *robot = new Robot();

    // Get the e-puck camera
    Camera *camera = robot->getCamera("camera");
    camera->enable(TIME_STEP); // Enable the camera

    // Get camera resolution
    int width = camera->getWidth();
    int height = camera->getHeight();

    std::cout << "Camera Initialized: " << width << "x" << height << std::endl;

    while (robot->step(TIME_STEP) != -1) {
        // Get camera image
        const unsigned char *image = camera->getImage();

        if (image) {
            // Convert Webots image to OpenCV format
            Mat frame(height, width, CV_8UC4, (void *)image);

            // Convert RGBA (Webots) to BGR (OpenCV)
            Mat frameBGR;
            cvtColor(frame, frameBGR, COLOR_RGBA2BGR);

            // Display the camera image using OpenCV
            imshow("e-Puck Camera", frameBGR);
            if (waitKey(1) == 27) break; // Exit on ESC key
        }
    }

    // Cleanup
    delete robot;
    return 0;
}
