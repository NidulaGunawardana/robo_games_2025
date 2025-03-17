import cv2
import numpy as np
from controller import Display



        
class CameraControl:
    ROBOT = None
    camera = None
    display = None

    def __init__(self, robot):
        self.ROBOT = robot
        self.camera = self.ROBOT.getDevice("range-finder")
        self.camera.enable(1)
        self.display = self.ROBOT.getDevice('display')

    def get_range(self):
        return self.camera.getRangeImage(data_type="buffer")
    
    def get_image(self):
        return self.camera.getImage()
    
    def display_image(self):
        image = self.get_range()
        image_size = self.camera.getWidth() * self.camera.getHeight()
        image = np.frombuffer(image, dtype=np.float32)
        if image.size != image_size:
            raise ValueError(f"Expected image size {image_size}, but got {image.size}")
        image = image.reshape((self.camera.getHeight(), self.camera.getWidth()))
        image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
        image = np.uint8(image)
        image = cv2.applyColorMap(image, cv2.COLORMAP_JET)
        
        # Display using Webots display
        ir = self.display.imageNew(int(self.camera.getWidth()), int(self.camera.getHeight()), image.tobytes(), Display.RGB)
        self.display.imagePaste(ir, 0, 0, False)
        self.display.imageDelete(ir)
        
        # Display using OpenCV
        cv2.imshow("Depth Image", image)
        cv2.waitKey(1)
