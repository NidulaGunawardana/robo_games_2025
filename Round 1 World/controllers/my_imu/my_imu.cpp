#include <webots/Robot.hpp>
#include <webots/InertialUnit.hpp>
#include <iostream>
#include <iomanip> // For formatted output
#include <string> // For using std::string

using namespace webots;
using namespace std;

string getDirection(Robot &robot) {

  const float M_PI = 3.14;
  
  // Get the simulation timestep
  int timeStep = (int)robot.getBasicTimeStep();

  // Get the IMU device
  InertialUnit *imu = robot.getInertialUnit("imu");
  if (!imu) {
    cerr << "IMU device not found!" << endl;
    return "Error: IMU not found";
  }

  // Enable the IMU
  imu->enable(timeStep);

  // Perform a single simulation step to get the IMU reading
  if (robot.step(timeStep) != -1) {
    // Get roll, pitch, yaw from the IMU
    const double *rpy = imu->getRollPitchYaw();
    double yaw = rpy[2];

    // Normalize yaw to [0, 2*PI] for easier handling
    if (yaw < 0) {
      yaw += 2 * M_PI;
    }

    // Determine direction based on yaw
    if ((yaw >= 7 * M_PI / 4 || yaw < M_PI / 4)) {
      return "West"; //West
    } else if (yaw >= M_PI / 4 && yaw < 3 * M_PI / 4) {
      return "South"; //South
    } else if (yaw >= 3 * M_PI / 4 && yaw < 5 * M_PI / 4) {
      return "East"; //East
    } else if (yaw >= 5 * M_PI / 4 && yaw < 7 * M_PI / 4) {
      return "North"; //North
    }
  }

  return "Error: Unable to determine direction";
}

int main() {
  // Initialize the robot
  Robot robot;

  while (true) {
    string direction = getDirection(robot);
    if (direction.find("Error") == string::npos) {
      cout << "Direction: " << direction << endl;
    } else {
      cerr << direction << endl;
      break;
    }
  }

  return 0;
}
