import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import numpy as np
from sklearn.svm import OneClassSVM

class DataHandlerNode(Node):
    def __init__(self):
        super().__init__('data_handler')
        self.subscription = self.create_subscription(
            Imu,  
            '/ground_station/imu_faulty',  
            self.sensor_callback,
            10
        )

        self.publisher = self.create_publisher(Float32, '/ground_station/imu_cleaned', 10)
        self.data_window = []
        self.window_size = 20  
        self.svm = OneClassSVM(nu=0.1, kernel='rbf', gamma='scale')  # SVM for outlier detection
    
    def sensor_callback(self, msg):
        data_point = msg.angular_velocity.x  # Use angular velocity x for processing
        self.data_window.append(data_point)
        if len(self.data_window) > self.window_size:
            self.data_window.pop(0)
        
        cleaned_data = self.process_data(np.array(self.data_window))
        if cleaned_data is not None and len(cleaned_data) > 0:
            cleaned_msg = Float32()
            cleaned_msg.data = cleaned_data[-1]
            self.publisher.publish(cleaned_msg)
            self.get_logger().info(f'Published Cleaned Data: {cleaned_msg.data}')
    
    def process_data(self, data):
        if len(data) < 5:
            return data  # Not enough data for processing
        
        # Outlier detection
        threshold = np.mean(data) + 3 * np.std(data)
        outliers = np.abs(data - np.mean(data)) > threshold
        data[outliers] = np.mean(data)  # Replace outliers with mean
        
        # Noise correction (simple moving average)
        smoothed_data = np.convolve(data, np.ones(3)/3, mode='valid')
        
        # Missing data handling (linear interpolation)
        if np.any(np.isnan(smoothed_data)):
            smoothed_data = np.nan_to_num(smoothed_data, nan=np.nanmean(smoothed_data))
        
        return smoothed_data

def main(args=None):
    rclpy.init(args=args)
    node = DataHandlerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
