import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# from std_msgs.msg import 
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class CameraStreamCollector(Node):
    def __init__(self):
        super().__init__('camera_stream_collector')        
    
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("CameraStreamCollector has started")
        self.display_visualization = True

        # self.publisher = self.create_publisher(String, 'camera_mask', 10)


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

        # self.find_red_cube(cv_image)
        self.find_colored_cubes(cv_image)
        

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
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
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

    def find_red_cube(self, cv_image):
        """
        Example to find red cube 
        """
        # BGR image -> HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Set the HSV range for red.
        # Note: red hue spans the low and high end of the hue circle
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

    def find_colored_cubes(self, cv_image):
        """
            Method that extracts colored cubes from the image input.
            
        """

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # WARNING: THIS IS ONLY GOOD FOR GAZEBO TESTING, NEEDS TO BE IMPROVED
        # Although this allows all colours, it weeds out the low saturation pixels (gazebo background)
        mask_color = cv2.inRange(hsv, (0, 50, 0), (179, 255, 255))
        blurred = cv2.GaussianBlur(mask_color, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # https://docs.opencv.org/4.x/d4/d73/tutorial_py_contours_begin.html

        for cnt in contours:
            # Approx each contour to a polygon
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            mask = np.zeros(cv_image.shape[:2], dtype="uint8")
            cv2.drawContours(mask, [approx], -1, 255, -1)
            mean_val = cv2.mean(cv_image, mask=mask)[:3]
            avg_colour_bgr = np.uint8([[list(mean_val)]])
            avg_colour_hsv = cv2.cvtColor(avg_colour_bgr, cv2.COLOR_BGR2HSV)[0][0]

            color_label = self.get_color_name(avg_colour_hsv)
            
            # Bounding rectangle for labelling
            x, y, w, h = cv2.boundingRect(approx)
            # Draw contour and draw the color label
            label_text_color_bgr = (int(avg_colour_bgr[0][0][0]), int(avg_colour_bgr[0][0][1]), int(avg_colour_bgr[0][0][2]))
            cv2.drawContours(cv_image, [approx], -1, (0, 255, 0), 2)
            cv2.putText(cv_image, color_label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, label_text_color_bgr, 2)
        

        # FOR VISUALIZATION
        if self.display_visualization:
            # cv2.imshow("Color Mask", mask_color)
            # cv2.imshow("Edges", edges)
            cv2.imshow("Result: Detected Cubes", cv_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()