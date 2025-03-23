# Standard imports
import cv2
import numpy as np
import freenect
import KobukiDriver as kobuki
import time

#robot specific function
def get_depth_and_rgb():
    depth, timestamp = freenect.sync_get_depth()
    rgb, timestamp = freenect.sync_get_video()
    return depth, rgb

# Blob detecting function: returns keypoints and mask
# return keypoints, reversemask
def blob_detect(image,                  #-- The frame (cv standard)
                hsv_min,                #-- minimum threshold of the hsv filter [h_min, s_min, v_min]
                hsv_max,                #-- maximum threshold of the hsv filter [h_max, s_max, v_max]
                blur=0,                 #-- blur value (default 0)
                blob_params=None,       #-- blob parameters (default None)
                search_window=None,     #-- window where to search as [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
                imshow=False
               ):

    # Blur image to remove noise
    if blur > 0: 
        image    = cv2.blur(image, (blur, blur))
        # Show result
        if imshow:
            cv2.imshow("Blur", image)
            cv2.waitKey(0)
        
    #- Search window
    if search_window is None: search_window = [0.0, 0.0, 1.0, 1.0]
    
    #- Convert image from BGR to HSV
    hsv     = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    #- Apply HSV threshold
    mask    = cv2.inRange(hsv,hsv_min, hsv_max)
    
    #- Show HSV Mask
    if imshow:
        cv2.imshow("HSV Mask", mask)
    
    #- dilate makes the in range areas larger
    mask = cv2.dilate(mask, None, iterations=2)
    #- Show HSV Mask
    if imshow:
        cv2.imshow("Dilate Mask", mask)   
        cv2.waitKey(0)
        
    mask = cv2.erode(mask, None, iterations=2)
    
    #- Show dilate/erode mask
    if imshow:
        cv2.imshow("Erode Mask", mask)
        cv2.waitKey(0)
    
    #- Cut the image using the search mask
    mask = apply_search_window(mask, search_window)
    
    if imshow:
        cv2.imshow("Searching Mask", mask)
        cv2.waitKey(0)

    #- build default blob detection parameters, if none have been provided
    if blob_params is None:
        # Set up the SimpleBlobdetector with default parameters.
        params = cv2.SimpleBlobDetector_Params()
         
        # Change thresholds
        params.minThreshold = 30;
        params.maxThreshold = 100;
         
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 750
        params.maxArea = 60000
         
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.5
         
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.5
         
        # Filter by Inertia
        params.filterByInertia =True
        params.minInertiaRatio = 0.5
         
    else:
        params = blob_params     

    #- Apply blob detection
    detector = cv2.SimpleBlobDetector_create(params)

    # Reverse the mask: blobs are black on white
    reversemask = 255-mask
    
    if imshow:
        cv2.imshow("Reverse Mask", reversemask)
        cv2.waitKey(0)
        
    keypoints = detector.detect(reversemask)

    return keypoints, reversemask

#---------- Draw detected blobs: returns the image
#-- return(im_with_keypoints)
def draw_keypoints(image,                   #-- Input image
                   keypoints,               #-- CV keypoints
                   line_color=(0,0,255),    #-- line's color (b,g,r)
                   imshow=False             #-- show the result
                  ):
    
    #-- Draw detected blobs as red circles.
    #-- cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
    if imshow:
        # Show keypoints
        cv2.imshow("Keypoints", im_with_keypoints)
        
    return(im_with_keypoints)

#---------- Draw search window: returns the image
#-- return(image)
def draw_window(image,              #- Input image
                window_adim,        #- window in adimensional units
                color=(255,0,0),    #- line's color
                line=5,             #- line's thickness
                imshow=False        #- show the image
               ):
    
    rows = image.shape[0]
    cols = image.shape[1]
    
    x_min_px    = int(cols*window_adim[0])
    y_min_px    = int(rows*window_adim[1])
    x_max_px    = int(cols*window_adim[2])
    y_max_px    = int(rows*window_adim[3])  
    
    #-- Draw a rectangle from top left to bottom right corner
    image = cv2.rectangle(image,(x_min_px,y_min_px),(x_max_px,y_max_px),color,line)
    
    if imshow:
        # Show keypoints
        cv2.imshow("Keypoints", image)

    return(image)

#---------- Draw X Y frame
#-- return(image)
def draw_frame(image,
               dimension=0.3,      #- dimension relative to frame size
               line=2              #- line's thickness
    ):
    
    rows = image.shape[0]
    cols = image.shape[1]
    size = min([rows, cols])
    center_x = int(cols/2.0)
    center_y = int(rows/2.0)
    
    line_length = int(size*dimension)
    
    #-- X
    image = cv2.line(image, (center_x, center_y), (center_x+line_length, center_y), (0,0,255), line)
    #-- Y
    image = cv2.line(image, (center_x, center_y), (center_x, center_y+line_length), (0,255,0), line)
    
    return (image)

#---------- Apply search window: returns the image
#-- return(image)
def apply_search_window(image, window_adim=[0.0, 0.0, 1.0, 1.0]):
    rows = image.shape[0]
    cols = image.shape[1]
    x_min_px    = int(cols*window_adim[0])
    y_min_px    = int(rows*window_adim[1])
    x_max_px    = int(cols*window_adim[2])
    y_max_px    = int(rows*window_adim[3])    
    
    #--- Initialize the mask as a black image
    mask = np.zeros(image.shape,np.uint8)
    
    #--- Copy the pixels from the original image corresponding to the window
    mask[y_min_px:y_max_px,x_min_px:x_max_px] = image[y_min_px:y_max_px,x_min_px:x_max_px]   
    
    #--- return the mask
    return(mask)
    
#---------- Apply a blur to the outside search region
#-- return(image)
def blur_outside(image, blur=5, window_adim=[0.0, 0.0, 1.0, 1.0]):
    rows = image.shape[0]
    cols = image.shape[1]
    x_min_px    = int(cols*window_adim[0])
    y_min_px    = int(rows*window_adim[1])
    x_max_px    = int(cols*window_adim[2])
    y_max_px    = int(rows*window_adim[3])    
    
    #--- Initialize the mask as a black image
    mask    = cv2.blur(image, (blur, blur))
    
    #--- Copy the pixels from the original image corresponding to the window
    mask[y_min_px:y_max_px,x_min_px:x_max_px] = image[y_min_px:y_max_px,x_min_px:x_max_px]   
    
    
    
    #--- return the mask
    return(mask)
    
#---------- Obtain the camera relative frame coordinate of one single keypoint
#-- return(x,y)
def get_blob_relative_position(image, keyPoint):
    rows = float(image.shape[0])
    cols = float(image.shape[1])
    # print(rows, cols)
    center_x    = 0.5*cols
    center_y    = 0.5*rows
    # print(center_x)
    x = (keyPoint.pt[0] - center_x)/(center_x)
    y = (keyPoint.pt[1] - center_y)/(center_y)
    return(x,y)

def push(): # Pushing the ball out of the arena
    my_kobuki.move(-100,-100,0)
    time.sleep(0.8)
    my_kobuki.move(0,0,0)
    time.sleep(0.1)
    my_kobuki.move(300,300,0)
    time.sleep(0.5)
    my_kobuki.move(0,0,0)  

def grab_ball(): # Grabbing the ball

    ball_state = False # Ball hasn't grabbed untill now

    depth, frame = get_depth_and_rgb()
    # Capture frame-by-frame
    #ret, frame = cap.read()
    #-- Detect keypoints
    keypoints, iv = blob_detect(frame, blue_min, blue_max, blur=3, 
                                blob_params=None, search_window=window, imshow=False)
    
    #-- Draw search window
    frame  = draw_window(frame, window)

    #-- click ENTER on the image window to proceed
    draw_keypoints(frame, keypoints, imshow=True)
    #draw_keypoints(iv, keypoints, imshow=True)


    # Keeping the ball in the frame and go towards the ball
    if keypoints:
        x_co_list = []
        y_co_list = []
        i = 0 

        pos_ball_list = get_blob_relative_position(frame, keypoints[0])
        # for pos_ball_l in pos_ball_list:
        #     x_co_list.append([pos_ball_l[0], i])
        #     y_co_list.append([pos_ball_l[1], i])
        #     i += 1

        # x_co_list.sort(reverse=True)
        # y_co_list.sort(reverse=True)
        # pos_most_right = x_co_list[0][1]
        # pos_most_up = y_co_list[0][1]

        # if pos_most_up == pos_most_right:
        #     pos_most_right = x_co_list[1][1]

        er_x = pos_ball_list[0]
        er_y = pos_ball_list[1]
        # er_y = pos_ball_list[pos_most_right][1]
        # print(er_y)
        speed_right = 125-(50*er_x)                                                                                  
        speed_left = 125+(50*er_x)
        my_kobuki.move(speed_left,speed_right,0)
        # if er_y>-0.65: # Grab position needs to be adjusted
           
        # else:
        #     my_kobuki.move(125,125,0)
        #     time.sleep(0.4)
        #     ball_state = True
        if er_y<-0.65:
            ball_state = True
            print('Ball grabbed')

    return ball_state

def carry_ball(): # Carrying the ball

    mat_state = False # Ball hasn't grabbed untill now

    depth, frame = get_depth_and_rgb()
    # Capture frame-by-frame
    #ret, frame = cap.read()
    #-- Detect keypoints
    keypoints, iv = blob_detect(frame, blue_min, blue_max, blur=3, 
                                blob_params=None, search_window=window_mat, imshow=False)
    
    #-- Draw search window
    frame  = draw_window(frame, window)

    #-- click ENTER on the image window to proceed
    draw_keypoints(frame, keypoints, imshow=True)
    #draw_keypoints(iv, keypoints, imshow=True)


    # Keeping the ball in the frame and go towards the ball
    if keypoints:
        y_co_list = []
        i = 0 

        pos_ball_list = get_blob_relative_position(frame, keypoints[0])
        # for pos_ball_l in pos_ball_list:
        #     x_co_list.append([pos_ball_l[0], i])
        #     y_co_list.append([pos_ball_l[1], i])
        #     i += 1

        # x_co_list.sort(reverse=True)
        # y_co_list.sort(reverse=True)
        # pos_most_right = x_co_list[0][1]
        # pos_most_up = y_co_list[0][1]

        # if pos_most_up == pos_most_right:
        #     pos_most_right = x_co_list[1][1]

        er_x = pos_ball_list[0]
        er_y = pos_ball_list[1]
        # er_y = pos_ball_list[pos_most_right][1]
        # print(er_y)
        speed_right = 125-(50*er_x)                                                                                  
        speed_left = 125+(50*er_x)
        my_kobuki.move(speed_left,speed_right,0)
        # if er_y>-0.65: # Grab position needs to be adjusted

        # depth, frame = get_depth_and_rgb()
        # #ret, frame = cap.read()
        # #-- Detect keypoints
        # keypoints, iv = blob_detect(frame, blue_min, blue_max, blur=3, 
        #                             blob_params=None, search_window=window_mat, imshow=False)
        
        # #-- Draw search window
        # frame  = draw_window(frame, window)

        # #-- click ENTER on the image window to proceed
        # draw_keypoints(frame, keypoints, imshow=True)
        # #draw_keypoints(iv, keypoints, imshow=True)
        # while keypoints==False:
        #     my_kobuki.move(60,-60,0)
        #     time.sleep(0.1)
        #     # Capture frame-by-frame

        #     depth, frame = get_depth_and_rgb()
        #     #ret, frame = cap.read()
        #     #-- Detect keypoints
        #     keypoints, iv = blob_detect(frame, blue_min, blue_max, blur=3, 
        #                                 blob_params=None, search_window=window_mat, imshow=False)
            
        #     #-- Draw search window
        #     frame  = draw_window(frame, window)

        #     #-- click ENTER on the image window to proceed
        #     draw_keypoints(frame, keypoints, imshow=True)
        # my_kobuki.move(0,0,0)
        if er_y<-0.65:
            push()
            mat_state = True
            print('Ball released')
    return mat_state

def search(): # resetting and searching for the new ball
    # my_kobuki.move(-100,-100,0)
    time.sleep(1)
    # Capture frame-by-frame
    depth, frame = get_depth_and_rgb()
    #ret, frame = cap.read()
    #-- Detect keypoints
    keypoints, iv = blob_detect(frame, blue_min, blue_max, blur=3, 
                                blob_params=None, search_window=window_mat, imshow=False)
    
    #-- Draw search window
    frame  = draw_window(frame, window)

    #-- click ENTER on the image window to proceed
    draw_keypoints(frame, keypoints, imshow=True)

    while keypoints == False:
        my_kobuki.move(100,-100,0)
        time.sleep(0.1)
        #ret, frame = cap.read()
        depth, frame = get_depth_and_rgb()
        #-- Detect keypoints
        keypoints, iv = blob_detect(frame, blue_min, blue_max, blur=3, 
                                    blob_params=None, search_window=window_mat, imshow=False)
        
        #-- Draw search window
        frame  = draw_window(frame, window)

        #-- click ENTER on the image window to proceed
        draw_keypoints(frame, keypoints, imshow=True)
    my_kobuki.move(0,0,0)

if __name__=="__main__":

    # Defining a robot object
    my_kobuki = kobuki.Kobuki()

    #--- Define HSV limits
    blue_min = (69,61,43)
    blue_max = (255, 255, 255) 
    
    #--- Define area limit [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
    window = [0.1, 0.25, 0.9, 1] # Getting the window to fit within the arms

    window_mat = [0.1, 0.25, 0.9, 1]
    
    #cap = cv2.VideoCapture(0)

    while True:

        # push()
        # search()
        # grab_ball()
        # carry_ball()

        if grab_ball() == True:
           
            while True:
                if carry_ball() == True:
                    
                    
                  
                    break


        #-- press q to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
