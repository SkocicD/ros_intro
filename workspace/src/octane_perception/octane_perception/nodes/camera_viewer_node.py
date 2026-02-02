import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
from flask import Flask, Response, render_template_string
import io


class CameraViewerNode(Node):
    """Display depth and color camera feeds via web interface"""

    def __init__(self):
        super().__init__('camera_viewer_node')
        self.get_logger().info('Camera Viewer node started')

        self.bridge = CvBridge()
        self.depth_image = None
        self.color_image = None
        self.last_depth_time = None
        self.last_color_time = None
        self.latest_frame = None

        # Subscribe to camera topics
        self.depth_subscription = self.create_subscription(
            Image,
            'depth_camera/depth',
            self.depth_callback,
            10
        )

        self.color_subscription = self.create_subscription(
            Image,
            'depth_camera/color',
            self.color_callback,
            10
        )

        # Timer to generate display frames
        self.timer = self.create_timer(0.033, self.generate_frame)  # ~30 FPS

        # Start web server in separate thread
        self.start_web_server()

    def depth_callback(self, msg):
        """Receive depth images"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.last_depth_time = time.time()
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')

    def color_callback(self, msg):
        """Receive color images"""
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_color_time = time.time()
        except Exception as e:
            self.get_logger().error(f'Failed to convert color image: {e}')

    def start_web_server(self):
        """Start Flask web server in background thread"""
        import logging
        import os

        # Suppress Flask's default logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.WARNING)

        # Disable Flask reloader output
        os.environ['WERKZEUG_RUN_MAIN'] = 'true'

        app = Flask(__name__)

        HTML_TEMPLATE = '''
        <!DOCTYPE html>
        <html>
        <head>
            <title>Camera View</title>
            <style>
                body {
                    margin: 0;
                    background: #1a1a1a;
                    font-family: monospace;
                    color: white;
                }
                .container {
                    display: flex;
                    flex-direction: column;
                    align-items: center;
                    padding: 20px;
                }
                h1 { color: #4CAF50; }
                img {
                    max-width: 95vw;
                    height: auto;
                    border: 2px solid #4CAF50;
                }
                .info {
                    margin-top: 10px;
                    color: #888;
                }
            </style>
            <script>
                // Auto-reload if connection is lost
                window.addEventListener('load', function() {
                    var img = document.querySelector('img');
                    img.addEventListener('error', function() {
                        setTimeout(function() { img.src = img.src; }, 1000);
                    });
                });
            </script>
        </head>
        <body>
            <div class="container">
                <h1>Octane Perception - Camera View</h1>
                <img src="/video_feed" alt="Camera Feed">
                <p class="info">Green dot = Active | Red dot = Waiting for camera</p>
                <p class="info">Left: Color | Right: Depth</p>
            </div>
        </body>
        </html>
        '''

        @app.route('/')
        def index():
            return render_template_string(HTML_TEMPLATE)

        @app.route('/video_feed')
        def video_feed():
            return Response(self.generate_frames(),
                          mimetype='multipart/x-mixed-replace; boundary=frame')

        def run_flask():
            try:
                port = 5000
                self.get_logger().info(f'Starting web server on http://0.0.0.0:{port}')
                app.run(host='0.0.0.0', port=port, threaded=True, use_reloader=False, debug=False)
            except Exception as e:
                self.get_logger().error(f'Flask server error: {e}')
                # Try alternative port if 5000 is taken
                try:
                    alt_port = 5001
                    self.get_logger().info(f'Retrying on port {alt_port}')
                    app.run(host='0.0.0.0', port=alt_port, threaded=True, use_reloader=False, debug=False)
                except Exception as e2:
                    self.get_logger().error(f'Failed to start web server: {e2}')

        flask_thread = threading.Thread(target=run_flask, daemon=True)
        flask_thread.start()

    def generate_frames(self):
        """Generator for Flask to stream frames"""
        while True:
            try:
                if self.latest_frame is not None:
                    ret, buffer = cv2.imencode('.jpg', self.latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ret:
                        frame = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                else:
                    # Generate a placeholder frame
                    placeholder = self.create_status_image(640, 480, "Initializing...")
                    ret, buffer = cv2.imencode('.jpg', placeholder, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ret:
                        frame = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                time.sleep(0.033)
            except Exception as e:
                self.get_logger().error(f'Error generating frame: {e}')
                time.sleep(0.1)

    def create_status_image(self, width, height, text):
        """Create a blank image with status text"""
        img = np.zeros((height, width, 3), dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_size = cv2.getTextSize(text, font, 0.7, 2)[0]
        text_x = (width - text_size[0]) // 2
        text_y = (height + text_size[1]) // 2
        cv2.putText(img, text, (text_x, text_y), font, 0.7, (255, 255, 255), 2)
        return img

    def generate_frame(self):
        """Generate combined frame for display"""
        current_time = time.time()
        timeout = 2.0  # Consider data stale after 2 seconds

        # Check if data is stale or missing
        depth_active = (self.last_depth_time is not None and
                       (current_time - self.last_depth_time) < timeout)
        color_active = (self.last_color_time is not None and
                       (current_time - self.last_color_time) < timeout)

        # Create or use actual images
        if color_active and self.color_image is not None:
            color_display = self.color_image.copy()
            h, w = color_display.shape[:2]
        else:
            h, w = 480, 640
            color_display = self.create_status_image(w, h, "Waiting for camera...")

        if depth_active and self.depth_image is not None:
            # Normalize depth for visualization
            depth_normalized = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_display = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
            h_d, w_d = depth_display.shape[:2]
            # Resize to match color image height
            depth_display = cv2.resize(depth_display, (int(w_d * h / h_d), h))
        else:
            depth_display = self.create_status_image(w, h, "Waiting for camera...")

        # Add status indicators
        status_color = (0, 255, 0) if color_active else (0, 0, 255)
        status_depth = (0, 255, 0) if depth_active else (0, 0, 255)

        cv2.circle(color_display, (20, 20), 10, status_color, -1)
        cv2.circle(depth_display, (20, 20), 10, status_depth, -1)

        # Combine side by side
        self.latest_frame = np.hstack((color_display, depth_display))


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
