import os
import cv2
import pyrealsense2 as rs
import numpy as np
from PIL import Image
import time

class RealsenseImageSaver:
    def __init__(self, pipeline, save_directory):
        self.pipeline = pipeline
        self.save_directory = save_directory
        self.frame_count = 0

    def start_stream(self):
        self.pipeline.stop()
        self.pipeline.start()

    def stop_stream(self):
        self.pipeline.stop()

    def save_frames(self, num_frames=8, prefix=''):
        try:
            self.start_stream()
            start_time = time.time()
            while self.frame_count < num_frames:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                image = Image.fromarray(color_image)
                image_path = os.path.join(self.save_directory, f'{prefix}_image_{self.frame_count}.jpg')
                image.save(image_path)
                self.frame_count += 1
                print(f'Saved {image_path}')

                # Calculate remaining time and wait to ensure even distribution
                elapsed_time = time.time() - start_time
                print(f"Time elapsed: {elapsed_time:.2f} seconds")
        finally:
            self.stop_stream()

def main():
    
    # Initialize Intel RealSense pipeline
    pipeline = rs.pipeline()

    # Configure the pipeline to stream color images
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    # Save directory
    save_directory = 'captured_images'

    # Create FrameSaver instance
    frame_saver = RealsenseImageSaver(pipeline, save_directory)
    
    # Save frames
    frame_saver.save_frames(num_frames=8, prefix='test')

if __name__ == '__main__':
    main()
