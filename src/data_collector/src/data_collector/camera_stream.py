import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class CameraStreamCollector(Node):
    def __init__(self):
        super().__init__('camera_stream_collector')        
    
        self.subscription = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("CameraStreamCollector has started")

        # this is for the ORB feature example
        # self.set_image_reference('red_cube.jpg')


    def image_callback(self, msg) -> None:
        """
        Main Method that is called to process the OpenCV image data
        """
        try:
            # ROS Image message -> OpenCV image (BGR8 encoding)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error("[***ERROR***] CvBridge Error: %s" % str(e))
            return

        # Examples to play with

        self.find_red_cube(cv_image)
        # self.find_multiple_cubes(cv_image)
        

    def find_contours(self, cv_image) -> list:
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150) # opencv edge detection
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def draw_contour(self, mask, cv_image) -> None:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:  # Set this threshold based on the size of the cube in the image
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2) # Draw box that inscribes the detected red cube
                # self.get_logger().info(
                #     f"Detected at pixel location: x={x}, y={y}, width={w}, height={h}"
                # )

    def get_color_name(self, hsv_colour):
        """
        Input: HSV_Colour
        Returns:
            (str) : Colour of detected object
        """
        h, s, v = hsv_colour
        if v < 50: # hardcoded staturation and brighness
            return "Black"
        if s < 50:
            if v > 200:
                return "White"
            else:
                return "Gray"
        # hue -> colour name 
        if (h < 10) or (h >= 160):
            return "Red"
        elif 10 <= h < 25:
            return "Orange"
        elif 25 <= h < 35:
            return "Yellow"
        elif 35 <= h < 85:
            return "Green"
        elif 85 <= h < 135:
            return "Blue"
        elif 135 <= h < 160:
            return "Purple"
        else:
            return "Unknown"

    def set_image_reference(self, path : str):
        # load the reference image for ORB matching 
        self.reference_image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if self.reference_image is None:
            self.get_logger().error("Reference image not found!")
            self.reference_keypoints = None
            self.reference_descriptors = None
        else:
            # get ORB features
            orb = cv2.ORB_create()
            self.reference_keypoints, self.reference_descriptors = orb.detectAndCompute(self.reference_image, None)
            self.get_logger().info("Reference image loaded and features computed.")

    def find_red_cube(self, cv_image):
        """
        Example to find red cube 
        """
        # BGR image -> HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Set the HSV range for red.
        # Note: red hue spans the low and high end of the hue circle.
        lower_red_1 = np.array([0, 70, 50])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 70, 50])
        upper_red_2 = np.array([180, 255, 255])

        # Create two masks and combine them
        mask1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        self.draw_contour(red_mask, cv_image)
        
        cv2.imshow("Red", cv_image)
        cv2.waitKey(1)

    def find_multiple_cubes(self, cv_image):
        grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(grey, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
        edges = cv2.Canny(thresh, 50, 150) # opencv edge detection
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            # IMPORTANT: This approxes the contour to polygons and filter for quadrilaterals... not good for general stuff
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            area = cv2.contourArea(approx) 
            if len(approx) == 4 and area > 500:  # adjust this area threshold if needed
                mask = np.zeros(cv_image.shape[:2], dtype="uint8")  # create mask for the contour
                cv2.drawContours(mask, [approx], -1, 255, -1)
                mean_val = cv2.mean(cv_image, mask=mask)[:3] # get avg BGR colour inside the contour
                avg_colour_bgr = np.uint8([[list(mean_val)]]) # avg colour -> HSV
                avg_colour_hsv = cv2.cvtColor(avg_colour_bgr, cv2.COLOR_BGR2HSV)[0][0]
                color_label = self.get_color_name(avg_colour_hsv)

                x, y, w, h = cv2.boundingRect(approx) # bounding rectangle for placing the text label
                
                cv2.drawContours(cv_image, [approx], -1, (0, 255, 0), 2) # draw contour and label
                cv2.putText(cv_image, color_label, (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                # self.get_logger().info(
                #     f"Cube detected at (x={x}, y={y}, w={w}, h={h}) with color: {color_label}"
                # )
        # result
        # cv2.imshow("grey", grey)
        # cv2.imshow("blurred", blurred)
        cv2.imshow("thresh", thresh)
        cv2.imshow("Edges", edges)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()