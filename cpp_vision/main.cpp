#include <opencv2/opencv.hpp>
#include "image_processing.h"

using namespace cv;
using namespace std;

int main() {
    // Load image
    Mat image = imread("assets/image.jpg");
    if (image.empty()) {
        cout << "Could not open or find the image!" << endl;
        return -1;
    }

    // Convert to grayscale
    Mat gray = convertToGray(image);

    // Display images
    imshow("Original Image", image);
    imshow("Grayscale Image", gray);
    waitKey(0);
    return 0;
}
