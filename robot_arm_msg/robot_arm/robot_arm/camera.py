#!/usr/bin/python3

import cv2
import rclpy
from rclpy.node import Node
import mediapipe as mp
import time
import numpy as np
from time import sleep
import pyrealsense2 as rs
import math
from robot_arm_msg.msg import CameraDists

class handdetec:
    def __init__(self, mode=False, MaxHands=2, modelComplex=1, detection=0.75, tracking=0.75):
        self.mode = mode
        self.MaxHands = MaxHands
        self.complex = modelComplex
        self.detection = detection
        self.track = tracking

        # detecting the landmarks
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.MaxHands, modelComplex, self.detection, tracking)
        self.mpdraw = mp.solutions.drawing_utils

    # job is to find the hands
    def find_hands(self, color_img, draw=True):
        """Identifies what the hand is in front of teh camera then draws points on it"""
        self.RGBimg = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
        self.result = self.hands.process(self.RGBimg)
        angles = []

        if self.result.multi_hand_landmarks:
            for hand_landmarks in self.result.multi_hand_landmarks:
                if draw:
                    thumb_tip = (int(hand_landmarks.landmark[4].x * color_img.shape[1]), int(hand_landmarks.landmark[4].y * color_img.shape[0]))
                    fingers_center = (int((hand_landmarks.landmark[8].x + hand_landmarks.landmark[12].x +
                                           hand_landmarks.landmark[16].x + hand_landmarks.landmark[20].x) / 4 *
                                          color_img.shape[1]),
                                      int((hand_landmarks.landmark[8].y + hand_landmarks.landmark[12].y +
                                           hand_landmarks.landmark[16].y + hand_landmarks.landmark[20].y) / 4 *
                                          color_img.shape[0]))
                    wrist = (int(hand_landmarks.landmark[0].x * color_img.shape[1]), int(hand_landmarks.landmark[0].y * color_img.shape[0]))
                    #landmark for the wrist

                    #call the angles function
                    angles = self.calculate_angles(hand_landmarks)
                    for i, angle in enumerate(angles.values()):
                        #check if the angle is a float
                        if isinstance(angle, float):
                            # draw angle near the wrist
                            cv2.putText(color_img, f'Angles {i + 1}: {angle:.2f}',
                                        (10, color_img.shape[0] - 20 * (len(angles) - i)), #this places the text under fps
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (140, 0, 255), 1)

                    #Draws the points on fingers, thumb, and wrisr
                    cv2.circle(color_img, thumb_tip, 20, (255, 0, 0), cv2.FILLED)
                    cv2.circle(color_img, fingers_center, 25, (255, 0, 0), cv2.FILLED)
                    cv2.circle(color_img, wrist, 20, (0, 0, 255), cv2.FILLED)

                    #draws lines to connect the points
                    cv2.line(color_img, wrist, thumb_tip, (255, 255, 255), 3)  # Line from wrist to thumb
                    cv2.line(color_img, wrist, fingers_center, (255, 255, 255), 3)  # Line from wrist to fingers center
                    cv2.line(color_img, thumb_tip, fingers_center, (255, 255, 255), 3)  # Line from thumb to fingers center
        
        return color_img, angles


    # find the positions (we'll use to then output)
    # might need to change origin_ variables to the previous outputed coordinates instead of 0,0,0
    def find_position(self, color_img, depth_img, handNum=0):
        """Finds the position of the hand from the points on the hand"""
        lmlist = []

        if self.result.multi_hand_landmarks:
            myHand = self.result.multi_hand_landmarks[handNum]

            #coordinates fo the origin
            origin_x, origin_y, origin_z = 0.0, 0.0, 0.0

            physical_width = 1.0 #example in meters
            physical_height = 1.0
            resolution_width, resolution_height = color_img.shape[0], color_img.shape[1]

            for id, lm in enumerate(myHand.landmark):
                if id in [0, 4, 8, 12, 16, 20]:
                    x_color = int(lm.x * color_img.shape[1])
                    y_color = int(lm.y * color_img.shape[0])

                    if x_color >= depth_img.shape[1] or y_color >= depth_img.shape[0]:
                        continue
                    cx, cy, cz = x_color, y_color, depth_img[y_color][x_color]  # assuming depth is 0

                    real_world_x, real_world_y, real_world_z = self.pixels_to_meters(cx, cy, cz, physical_width,
                                                                                physical_height, resolution_width,
                                                                                resolution_height)

                    relative_x = real_world_x - origin_x
                    # these are from debugging print(relative_x)
                    relative_y = real_world_y - origin_y
                    #print(relative_y)
                    relative_z = real_world_z - origin_z
                    #print(relative_z)
                    lmlist.append([id, relative_x, relative_y, relative_z])

        return lmlist

    def calculate_angles(self, hand_landmarks):
        """Calculates the angles that the hand is moving on camera"""
        angles = {}
        # Thumb, index, middle, ring, pinky
        finger_indicies = [4, 8, 12, 16, 20]
        # from teh wrist to the joints of the fingers
        palm_indicies = [0, 1, 5, 9, 13, 17]

        #calc angles between palm and finger??
        for finger_indx in range(len(finger_indicies)):
            #take all te finger numbers
            tip = finger_indicies[finger_indx]
            base = palm_indicies[finger_indx]

            tip_landmark = hand_landmarks.landmark[tip]
            base_landmark = hand_landmarks.landmark[base]
            wrist_landmark = hand_landmarks.landmark[0]

            tip_vector = np.array([tip_landmark.x, tip_landmark.y, tip_landmark.z])
            base_vector = np.array([base_landmark.x, base_landmark.y, base_landmark.z])
            wrist_vector = np.array([wrist_landmark.x, wrist_landmark.y, wrist_landmark.z])

            #calculate the angle between the finger vectors and the wrists
            finger_wrist_vector = wrist_vector - base_vector
            dot_product = np.dot(tip_vector - base_vector, finger_wrist_vector)
            magnitude_prod = np.linalg.norm(tip_vector - base_vector) * np.linalg.norm(finger_wrist_vector)

            if magnitude_prod != 0:
                cos_ang = dot_product / magnitude_prod
                #need to avoid invalid input
                cos_ang = max(min(cos_ang, 1), -1)
                angle = np.arccos(cos_ang)
                angles[f'Finger_{finger_indx} + 1'] = np.degrees(angle)
            else:
                angles[f'Finger_{finger_indx + 1}'] = np.nan

        #calculate the angle of the wrist flex/extend
        wrist_tip = np.array([hand_landmarks.landmark[20].x, hand_landmarks.landmark[20].y, hand_landmarks.landmark[20].z])
        wrist_base = np.array([hand_landmarks.landmark[0].x, hand_landmarks.landmark[0].y, hand_landmarks.landmark[0].z])

        wrist_vector = wrist_tip - wrist_base
        # Assuming z-axis is upward
        reference_vector = np.array([0, 0, 1])
        wrist_angle = np.arccos(np.dot(wrist_vector, reference_vector) / (
                    np.linalg.norm(wrist_vector) * np.linalg.norm(reference_vector)))
        angles['Wrist_Flexion_Extension'] = np.degrees(wrist_angle)

        return angles

    # Converts x and y pixels to meters. Since the depth camera is another module it already returns distance in millimeters
    def pixels_to_meters(self, pixel_x, pixel_y, pixel_z, physical_width, physical_height,
                        resolution_width, resolution_height):
        X_FOV = 87
        Y_FOV = 58

        """ This converts the pixel output into meters"""
        pixel_size_horizontal = physical_width / resolution_width
        pixel_size_veritcal = physical_height / resolution_height

        real_world_x = pixel_x * pixel_size_horizontal #TODO: Change this to calculate the ratio and then calculate to real world values
        real_world_y = pixel_y * pixel_size_veritcal
        real_world_z = pixel_z 

        true_x_pos = self.normalize_position(X_FOV, real_world_x, real_world_z)
        true_y_pos = self.normalize_position(Y_FOV, real_world_y, real_world_z)

        return true_x_pos, true_y_pos, real_world_z
        # return real_world_x, real_world_y, real_world_z
    
    def normalize_position(self, fov, observed_x_or_y, real_world_z):
        real_x_or_y_value = real_world_z * math.tan(math.radians(fov / 2))

        ratio = real_x_or_y_value / observed_x_or_y



        return true_pos
    
class Camera(Node):
    def __init__(self):
        super().__init__('camera_pub')
        self.publisher_ = self.create_publisher(CameraDists, '/camera_dists', 10)
        self.i = 0
    
    def publish(self, message):
        self.publisher_.publish(message)
        self.get_logger().info(f"Publishing: {message.x_dist}, {message.y_dist}, {message.z_dist}, {message.r}, {message.p}, {message.y}")
    
def main():
    # RGB camera setup
    cTime = 0
    pTime = 0
    detector = handdetec()
    rclpy.init()
    camera_dists = Camera()
    message = CameraDists()

    # Depth camera setup 
    # TODO: Depth camera matching to RGB camera to find depth values for fingers
    pipeline = rs.pipeline()
    config = rs.config()
    detector = handdetec()

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    pTime = time.time()

    while True:
        cTime = time.time()
        if cTime - pTime > .5:
            pTime = cTime
        else:
            continue


        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        # class object
        detector = handdetec()
        color_img, hand_angle = detector.find_hands(color_image)
        lm_list = detector.find_position(color_img, depth_image)

        #print(lm_list[0][1] * 1000, lm_list[0][2] * 1000, lm_list[0][3])

        # make sure there is a hand detected
        if not lm_list == [] and not hand_angle == []:
            message.x_dist = (lm_list[0][1] * 1000) # convert to millimeters
            message.y_dist = (lm_list[0][2] * 1000) # convert to millimeters
            message.z_dist = lm_list[0][3] # already in millimeters
            #message.r = hand_angle["Finger_1 + 1"]
            #message.p = 0 #hand_angle["Wrist_Flexion_Extension"]
            #message.y = 0  
            message.closed = False;
            camera_dists.publish(message)
            pass
        
        # calculating time
        

        #cv2.putText(color_img, f'FPS: {int(fps)}', (10, 70), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1)
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.1), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                            interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('Hand Tracking', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Hand Tracking', images)
        if cv2.waitKey(20) & 0xFF == ord('q'):  #this will close the window
            break

    message.destroy_node()
    # cap.release()
    cv2.destroyAllWindows()

    pipeline.stop()


if __name__ == '__main__':
    main()
