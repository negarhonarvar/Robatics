#############################   Embed CNN  #############################   

import keras
import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

model = keras.models.load_model("../../model/location.keras")


def predict(sample_images):
    # Predict labels
    predicted = model.predict(sample_images)
    predicted_labels = np.argmax(predicted, axis=1)

    class_dict = {0: "T-shirt", 1: "pants", 2: "pullover", 3: "shoes", 4: "Bag"}
    # Display images and predictions
    fig, axes = plt.subplots(1, len(sample_images), figsize=(20, 2))
    for i, ax in enumerate(axes):
        ax.imshow(sample_images[i].reshape(28, 28), cmap="gray")
        ax.set_title(f"{class_dict.get(predicted_labels[i])}")
        ax.axis("off")
    plt.show()
        
    return predicted_labels


def resize_image(image, new_size):
    img_pil = Image.fromarray(image)
    img_28x28 = np.array(img_pil.resize(new_size))
    return img_28x28


n = 5
rows_up = [0, 63, 125, 20, 80, 72]
rows_down = [0, 44, 0, 90, 25, 30]
columns_up = [0, 150, 160, 235, 145, 105]
columns_down = [0, 118, 110, 35, 125, 167]

def increase_detail(img, img_name):
    laplacian_filtered = cv2.Laplacian(img, cv2.CV_64F)
    laplacian_filtered = cv2.convertScaleAbs(laplacian_filtered)
    enhanced_img = cv2.add(img, laplacian_filtered)
    img_new8_1 = enhanced_img.astype(np.uint8)
    cv2.imwrite(img_name, img_new8_1)
    return img_new8_1


def predict_one_box(i):
    im_gray = cv2.imread(f'../../datasets/box{i}/output_image{i}-1.jpg', cv2.IMREAD_GRAYSCALE)
    cv2.imwrite(f'../../datasets/box{i}/output_image{i}-1_gray_scale.jpg', im_gray)
    
    height, width = im_gray.shape
    im_gray_crop = im_gray[rows_up[i]:height - rows_down[i], columns_up[i]:width - columns_down[i]]
    cv2.imwrite(f'../../datasets/box{i}/output_image{i}-1_gray_scale_crop.jpg', im_gray_crop)
    
    min_filtered = cv2.erode(im_gray_crop, None)
    img_new1 = min_filtered.astype(np.uint8)
    cv2.imwrite(f'../../datasets/box{i}/output_image{i}-1_gray_scale_crop_min.jpg', img_new1)
    
    img_new8_1 = increase_detail(img_new1, f"../../datasets/box{i}/output_image{i}-1_gray_scale_crop_min_lap.jpg")
    
    im_gray_crop_resize = resize_image(img_new8_1, (28, 28))
    cv2.imwrite(f'../../datasets/box{i}/output_image{i}-1_gray_scale_crop_resize.jpg', im_gray_crop_resize)
    
    sample_images = [im_gray_crop_resize, im_gray_crop_resize]

    class_dict = {0: "T-shirt", 1: "pants", 2: "pullover", 3: "shoes", 4: "Bag"}
    box_dict = {"T-shirt": 1, "pants": 5, "pullover": 2, "shoes": 4, "Bag": 3}
    
    pred = predict(np.array(sample_images))
    pred_title = class_dict[pred[0]]
    
    return box_dict[pred_title]



#############################   Embed CNN  #############################   


from controller import Robot
import sys

import numpy as np

webots_path = 'D:\\Webots\\Webots\\lib\\controller\\python'
sys.path.append(webots_path)

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

destination = 5
class Mavic (Robot):
    # Constants, empirically found.
    K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
    # Vertical offset where the robot actually targets to stabilize itself.
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0        # P constant of the vertical PID.
    K_ROLL_P = 50.0           # P constant of the roll PID.
    K_PITCH_P = 30.0          # P constant of the pitch PID.

    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    # Precision between the target position and the robot position in meters
    target_precision = 0.5

    def __init__(self):
        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        # Get and enable devices.
        self.led0 = self.getDevice("front left led")
        self.led1 = self.getDevice("front right led")
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)
        self.landing = False

        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        self.camera_pitch_motor = self.getDevice("camera pitch")
        self.camera_pitch_motor.setPosition(0.7)
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 0
        self.itr = 1


    def set_position(self, pos):
        """
        Set the new absolute position of the robot
        Parameters:
            pos (list): [X,Y,Z,yaw,pitch,roll] current absolute position and angles
        """
        self.current_pose = pos

    def adjust_yaw_to_target(self, target_yaw):
        """
        Adjust the drone's yaw until it aligns with the target yaw.
        Args:
            target_yaw (float): Target yaw angle in radians.
        Returns:
            float: The yaw disturbance to apply.
        """
        current_yaw = self.current_pose[5]  # Current yaw value
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        # Normalize the yaw angles to [-pi, pi]
        yaw_error = (target_yaw - yaw + np.pi) % (2 * np.pi) - np.pi

        # Calculate the yaw disturbance based on the yaw error
        return -self.MAX_YAW_DISTURBANCE * yaw_error / np.pi
    
     

    
    def move_to_target(self, waypoints, verbose_movement=False, verbose_target=False):
        """
        Move the robot to the given coordinates
        Parameters:
            waypoints (list): list of X,Y coordinates
            verbose_movement (bool): whether to print remaning angle and distance or not
            verbose_target (bool): whether to print targets or not
        Returns:
            yaw_disturbance (float): yaw disturbance (negative value to go on the right)
            pitch_disturbance (float): pitch disturbance (negative value to go forward)
        """

        if self.target_position[0:2] == [0, 0]:  # Initialization
            self.target_position[0:2] = waypoints[0]
            if verbose_target:
                print("First target: ", self.target_position[0:2])

        # if the robot is at the position with a precision of target_precision
        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):

            self.target_index += 1
            if self.target_index > len(waypoints) - 1:
                self.target_index = 0
            self.target_position[0:2] = waypoints[self.target_index]
            if verbose_target:
                while self.step(self.time_step) != -1:
                    yaw_disturbance = 1.3 #self.adjust_yaw_to_target(np.pi / 2)
                    roll, pitch, yaw = self.imu.getRollPitchYaw()
                    print(yaw_disturbance,10*'-',yaw)
                    if abs(yaw) > 3.13:

                        n = 1
                        for j in range(1, n + 1):
                            file_name = f'../../datasets/box{self.itr}/output_image{self.itr}-{j}.jpg'
                            self.camera.saveImage(file_name, 100)
                            print(f"save {file_name}")
                        
                        target_lable = predict_one_box(self.itr) # the lable of this image : output of CNN
                        if target_lable == destination:
                            self.landing = True
                            return 2,2
                        
                        self.itr += 1
                        break
                    
                        
                    roll, pitch, yaw = self.imu.getRollPitchYaw()
                    x_pos, y_pos, altitude = self.gps.getValues()
                    roll_input = self.K_ROLL_P * clamp(roll, -1, 1) 
                    pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) 
                    yaw_input = yaw_disturbance
                    clamped_difference_altitude = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
                    vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

                    front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
                    front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
                    rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
                    rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

                    self.front_left_motor.setVelocity(front_left_motor_input)
                    self.front_right_motor.setVelocity(-front_right_motor_input)
                    self.rear_left_motor.setVelocity(-rear_left_motor_input)
                    self.rear_right_motor.setVelocity(rear_right_motor_input)
                
                print("Target reached! New target: ", self.target_position[0:2])

        # This will be in [-pi;pi]
        self.target_position[2] = np.arctan2(
            self.target_position[1] - self.current_pose[1], self.target_position[0] - self.current_pose[0])
        # This is now in [-2pi;2pi]
        angle_left = self.target_position[2] - self.current_pose[5]
        # Normalize turn angle to [-pi;pi]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if (angle_left > np.pi):
            angle_left -= 2 * np.pi

        # Turn the robot to the left or to the right according the value and the sign of angle_left
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        # non proportional and decreasing function
        pitch_disturbance = clamp(
            np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        if verbose_movement:
            distance_left = np.sqrt(((self.target_position[0] - self.current_pose[0]) ** 2) + (
                (self.target_position[1] - self.current_pose[1]) ** 2))
            print("remaning angle: {:.4f}, remaning distance: {:.4f}".format(
                angle_left, distance_left))
        return yaw_disturbance, pitch_disturbance

    def run(self):
        t1 = self.getTime()

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0

        dx = 0.35
        dy = 0.34
        # Specify the patrol coordinates
        goal = [(-5, 4), (-3, -2), (3, -3), (5, 0), (2, 5)]
        waypoints = [(-5+dx , 4+dy ), (-2.74, -1.65), (2.8, -3), (4.5, -0.5), (2.5, 5.5)]
        # target altitude of the robot in meters
        self.target_altitude = 2.8

        while self.step(self.time_step) != -1:

            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            if not(self.landing) and altitude > self.target_altitude - 1:
                # as soon as it reach the target altitude, compute the disturbances to go to the given waypoints.
                if self.getTime() - t1 > 0.1:
                    yaw_disturbance, pitch_disturbance = self.move_to_target(
                        waypoints,False,True)
                    t1 = self.getTime()
                    
            if self.landing:
                self.target_altitude = 0
                
            if self.landing and altitude < 0.1:
                print("THE END!")
                self.led0.set(1)
                self.led1.set(1)

                motors = [self.front_left_motor, self.front_right_motor,
                          self.rear_left_motor, self.rear_right_motor]
                for motor in motors:
                    motor.setPosition(0)
                    motor.setVelocity(0)
                exit()

            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_difference_altitude = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(-front_right_motor_input)
            self.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.rear_right_motor.setVelocity(rear_right_motor_input)


# To use this controller, the basicTimeStep should be set to 8 and the defaultDamping
# with a linear and angular damping both of 0.5
robot = Mavic()
robot.run()