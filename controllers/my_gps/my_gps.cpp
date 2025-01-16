#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <iostream>
#include <cmath>

using namespace webots;
using namespace std;

// Function to calculate the cell number based on GPS coordinates
int calculateCell(double x, double y) {
    // Matrix boundaries and cell dimensions
    const double leftX = 1.25; // Rightmost X value (X-axis from right to left)
    const double topY = -1.25;   // Topmost Y value (Y-axis from top to bottom)
    const double cellWidth = 0.25;
    const double cellHeight = 0.25;
    const int columns = 10; // Number of columns in the matrix (2.5 / 0.25)

    // Calculate column and row indices
    int column = static_cast<int>((leftX - x) / cellWidth) + 1; // Reverse X direction
    int row = static_cast<int>((y - topY) / cellHeight) + 1;     // Reverse Y direction

    // Ensure indices are within valid bounds
    if (column < 1 || column > columns || row < 1 || row > columns) {
        return -1; // Out of bounds
    }

    // Calculate cell number (row-major order, left-to-right, top-to-bottom)
    int cellNumber = (row - 1) * columns + column;
    return cellNumber;
}

int main() {
    // Create Robot instance
    Robot robot;

    // Get the time step of the simulation
    int timeStep = static_cast<int>(robot.getBasicTimeStep());

    // Get the GPS device
    GPS *gps = robot.getGPS("gps");
    if (gps == nullptr) {
        cerr << "Error: GPS device not found. Please check the robot configuration." << endl;
        return -1;
    }

    // Enable the GPS sensor
    gps->enable(timeStep);

    cout << "Starting simulation loop..." << endl;

    // Simulation loop
    while (robot.step(timeStep) != -1) {
        // Get GPS coordinates
        const double *gpsValues = gps->getValues();
        double x = gpsValues[0];
        double y = gpsValues[1]; // Using Z as the Y-axis in the matrix

        // Calculate cell number
        int cellNumber = calculateCell(x, y);

        if (cellNumber == -1) {
            // cout << "The robot is out of bounds!" << endl;
            // cout << "GPS Coordinates: "
             // << "X = " << gpsValues[0]
             // << ", Y = " << gpsValues[1]
             // << ", Z = " << gpsValues[2]
             // << endl;
    
        } else {
            cout << "GPS Coordinates: X = " << x << ", Y = " << y
                 << " | Cell Number: " << cellNumber << endl;
        }
    }

    return 0;
}
