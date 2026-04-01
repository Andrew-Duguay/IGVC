#!/usr/bin/env python3
"""Lane detection candidate methods for benchmarking.

Each method subscribes to a camera image, detects lane lines, projects to
ground plane, and publishes /lane_points as PointCloud2 in base_footprint frame.

Methods are selected via the 'method' parameter:
- hsv_threshold: HSV white line detection (existing approach)
- brightness: Simple grayscale brightness threshold
- canny_hough: Canny edge detection + Hough line projection
- adaptive: Adaptive threshold for varying lighting
- lab_lightness: LAB color space L-channel threshold
- saturation_filter: Low-saturation + high-value (white detection)
- sobel_gradient: Sobel gradient magnitude threshold
- combined_vote: Multi-method voting (brightness + HSV + LAB)
- morpho_skeleton: Brightness threshold + morphological skeleton
- clahe_enhanced: CLAHE contrast enhancement + brightness threshold
"""

import cv2
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class LaneCandidateNode(Node):
    def __init__(self):
        super().__init__('lane_candidate_node')

        self.declare_parameter('method', 'hsv_threshold')
        self.declare_parameter('image_topic', '/left_camera/image_raw')
        self.declare_parameter('frame_skip', 2)

        # Camera intrinsics + mounting
        self.declare_parameter('rgb_fx', 349.2)
        self.declare_parameter('rgb_fy', 349.2)
        self.declare_parameter('rgb_cx', 320.0)
        self.declare_parameter('rgb_cy', 240.0)
        self.declare_parameter('camera_height', 0.27)
        self.declare_parameter('camera_forward_offset', 0.38)
        self.declare_parameter('camera_lateral_offset', 0.0)
        self.declare_parameter('camera_pitch', 0.0)   # radians, positive = tilted down
        self.declare_parameter('camera_yaw', 0.0)     # radians, positive = tilted left
        self.declare_parameter('min_range', 0.4)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('roi_top_row', 160)
        self.declare_parameter('subsample_stride', 4)

        # Method-specific params
        self.declare_parameter('brightness_thresh', 200)
        self.declare_parameter('hsv_low_v', 230)
        self.declare_parameter('hsv_high_s', 25)
        self.declare_parameter('adaptive_block_size', 51)
        self.declare_parameter('adaptive_c', -20)
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('lab_l_thresh', 200)
        self.declare_parameter('clahe_clip', 3.0)
        self.declare_parameter('morph_kernel_size', 3)

        self.bridge = CvBridge()
        self.frame_count = 0

        method = self.get_parameter('method').value
        self._detect_fn = {
            'hsv_threshold': self._detect_hsv,
            'brightness': self._detect_brightness,
            'canny_hough': self._detect_canny,
            'adaptive': self._detect_adaptive,
            'lab_lightness': self._detect_lab,
            'saturation_filter': self._detect_saturation,
            'sobel_gradient': self._detect_sobel,
            'combined_vote': self._detect_combined_vote,
            'morpho_skeleton': self._detect_morpho_skeleton,
            'clahe_enhanced': self._detect_clahe,
        }.get(method, self._detect_brightness)

        image_topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, image_topic, self._image_cb, 10)
        self.declare_parameter('output_topic', '/candidate/lane_points')
        output_topic = self.get_parameter('output_topic').value
        self.pc_pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.mask_pub = self.create_publisher(Image, '/candidate/lane_mask', 10)
        self.debug_pub = self.create_publisher(Image, '/candidate/debug_image', 10)

        self.get_logger().info(f'Lane candidate node started (method={method})')

    def _p(self, name):
        return self.get_parameter(name).value

    def _image_cb(self, msg):
        self.frame_count += 1
        if self.frame_count % self._p('frame_skip') != 0:
            return

        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        roi_top = self._p('roi_top_row')

        # Run detection method → binary mask
        mask = self._detect_fn(bgr)

        # Filter out wide blobs (obstacles) — keep only thin features (lane lines)
        mask = self._filter_wide_blobs(mask)

        # Publish debug mask
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        # Publish annotated debug image
        self._publish_debug_image(bgr, mask, roi_top, msg.header)

        # Extract and project white pixels from ROI
        roi_mask = mask[roi_top:, :]
        vs, us = np.where(roi_mask > 0)
        if len(vs) == 0:
            return
        vs = vs + roi_top

        stride = self._p('subsample_stride')
        vs = vs[::stride]
        us = us[::stride]
        if len(vs) == 0:
            return

        points = self._project_to_ground(vs, us)
        if len(points) == 0:
            return

        header = Header()
        # stamp=0 tells RViz to use latest TF (matches fake_lanes behavior)
        header.frame_id = 'base_footprint'
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg = point_cloud2.create_cloud(header, fields, points)
        self.pc_pub.publish(pc_msg)

    def _project_to_ground(self, vs, us):
        """Flat ground-plane projection from pixel coords to body frame.

        When camera_pitch and camera_yaw are both 0, this reduces to the
        original simple projection (t = cam_h / y_norm). When nonzero,
        pixel rays are rotated by the camera orientation before intersecting
        the ground plane.
        """
        fx = self._p('rgb_fx')
        fy = self._p('rgb_fy')
        cx = self._p('rgb_cx')
        cy = self._p('rgb_cy')
        cam_h = self._p('camera_height')
        cam_fwd = self._p('camera_forward_offset')
        cam_lat = self._p('camera_lateral_offset')
        pitch = self._p('camera_pitch')
        yaw = self._p('camera_yaw')
        min_r = self._p('min_range')
        max_r = self._p('max_range')

        y_norm = (vs.astype(np.float32) - cy) / fy
        x_norm = (us.astype(np.float32) - cx) / fx

        if abs(pitch) < 0.001 and abs(yaw) < 0.001:
            # Original simple projection (no rotation)
            valid = y_norm > 0.01
            y_norm = y_norm[valid]
            x_norm = x_norm[valid]
            if len(y_norm) == 0:
                return []
            t = cam_h / y_norm
            X = t + cam_fwd
            Y = -x_norm * t + cam_lat
        else:
            # Rotated projection: build rays in camera frame, rotate to body
            # Camera frame: X=forward, Y=left, Z=up
            # From optical frame: cam_x=1, cam_y=-x_norm, cam_z=-y_norm
            rx = np.ones_like(x_norm)
            ry = -x_norm
            rz = -y_norm

            # Ry(pitch): rotate about Y (nose down for positive pitch)
            cp, sp = math.cos(pitch), math.sin(pitch)
            rx2 = cp * rx + sp * rz
            ry2 = ry
            rz2 = -sp * rx + cp * rz

            # Rz(yaw): rotate about Z (nose left for positive yaw)
            cy_r, sy_r = math.cos(yaw), math.sin(yaw)
            rx3 = cy_r * rx2 - sy_r * ry2
            ry3 = sy_r * rx2 + cy_r * ry2
            rz3 = rz2

            # Ground intersection: camera at (cam_fwd, cam_lat, cam_h)
            # Ray hits z=0 when: cam_h + t * rz3 = 0
            valid = rz3 < -0.001
            rx3, ry3, rz3 = rx3[valid], ry3[valid], rz3[valid]
            if len(rx3) == 0:
                return []
            t = -cam_h / rz3
            X = cam_fwd + t * rx3
            Y = cam_lat + t * ry3

        Z = np.zeros_like(X)
        ranges = np.sqrt(X * X + Y * Y)
        in_range = (ranges >= min_r) & (ranges <= max_r)
        X, Y, Z = X[in_range], Y[in_range], Z[in_range]

        if len(X) == 0:
            return []
        return np.stack([X, Y, Z], axis=-1).tolist()

    # ─── Debug visualization ────────────────────────────────────────────

    def _publish_debug_image(self, bgr, mask, roi_top, header):
        """Publish camera image with lane detection annotations overlaid."""
        debug = bgr.copy()

        # Draw ROI line
        cv2.line(debug, (0, roi_top), (debug.shape[1], roi_top), (0, 255, 255), 1)
        cv2.putText(debug, 'ROI', (5, roi_top - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)

        # Overlay detected lane pixels in green
        lane_overlay = np.zeros_like(debug)
        lane_overlay[mask > 0] = (0, 255, 0)
        debug = cv2.addWeighted(debug, 1.0, lane_overlay, 0.4, 0)

        # Draw bounding boxes around connected components
        n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8)
        for i in range(1, n_labels):
            x = stats[i, cv2.CC_STAT_LEFT]
            y = stats[i, cv2.CC_STAT_TOP]
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            area = stats[i, cv2.CC_STAT_AREA]
            if area < 20:
                continue

            # Color: green for lane-like, red for rejected blobs
            bbox_area = max(w * h, 1)
            compactness = area / bbox_area
            if area > 2000 and compactness > 0.3:
                color = (0, 0, 255)  # red = rejected
                label = 'REJ'
            else:
                color = (0, 255, 0)  # green = lane
                label = 'LANE'

            cv2.rectangle(debug, (x, y), (x + w, y + h), color, 1)
            cv2.putText(debug, f'{label} {area}px',
                        (x, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)

        # Status text
        n_lane_px = np.sum(mask[roi_top:] > 0)
        cv2.putText(debug, f'Lane px: {n_lane_px}',
                    (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_msg.header = header
        self.debug_pub.publish(debug_msg)

    # ─── Blob filtering ────────────────────────────────────────────────

    def _filter_wide_blobs(self, mask):
        """Remove connected components that are too wide to be lane lines.

        Lane lines are thin (~5-20px wide). Obstacles like barrels appear as
        wide blobs (50+ px). Filter by bounding box aspect ratio and width.
        """
        n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        filtered = np.zeros_like(mask)
        max_blob_area = 2000  # pixels — barrels are large blobs, lane lines are smaller
        max_compactness = 0.3  # area / (w*h) — lane lines are sparse, blobs are dense
        min_blob_area = 20    # filter out tiny noise speckles

        for i in range(1, n_labels):  # skip background (0)
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            area = stats[i, cv2.CC_STAT_AREA]
            bbox_area = max(w * h, 1)
            compactness = area / bbox_area

            # Reject: tiny noise, OR large compact blobs (obstacles)
            if area < min_blob_area:
                continue  # too small = noise
            if area < max_blob_area or compactness < max_compactness:
                filtered[labels == i] = 255

        return filtered

    # ─── Detection methods ────────────────────────────────────────────

    def _morph_clean(self, mask):
        """Apply morphological open+close to clean noise."""
        k = self._p('morph_kernel_size')
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _detect_hsv(self, bgr):
        """HSV white line detection (existing approach)."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        low = np.array([0, 0, self._p('hsv_low_v')])
        high = np.array([180, self._p('hsv_high_s'), 255])
        mask = cv2.inRange(hsv, low, high)
        return self._morph_clean(mask)

    def _detect_brightness(self, bgr):
        """Simple grayscale brightness threshold."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        thresh = self._p('brightness_thresh')
        _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)
        return self._morph_clean(mask)

    def _detect_canny(self, bgr):
        """Canny edge detection projected to ground."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, self._p('canny_low'), self._p('canny_high'))
        # Dilate edges slightly to get more coverage
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges = cv2.dilate(edges, kernel, iterations=1)
        return edges

    def _detect_adaptive(self, bgr):
        """Adaptive threshold for varying lighting conditions."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        block = self._p('adaptive_block_size')
        c = self._p('adaptive_c')
        mask = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, block, c)
        return self._morph_clean(mask)

    def _detect_lab(self, bgr):
        """LAB color space — L channel is more robust to shadows than HSV V."""
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        l_channel = lab[:, :, 0]
        thresh = self._p('lab_l_thresh')
        _, mask = cv2.threshold(l_channel, thresh, 255, cv2.THRESH_BINARY)
        return self._morph_clean(mask)

    def _detect_saturation(self, bgr):
        """Low saturation + high value = white detection in HSV."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # White: any hue, low saturation, high value
        low = np.array([0, 0, self._p('hsv_low_v')])
        high = np.array([180, 30, 255])  # tighter saturation than hsv_threshold
        mask = cv2.inRange(hsv, low, high)
        return self._morph_clean(mask)

    def _detect_sobel(self, bgr):
        """Sobel gradient magnitude — detects edges of lane lines."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        mag = np.sqrt(sobelx**2 + sobely**2)
        mag = np.uint8(np.clip(mag / mag.max() * 255, 0, 255)) if mag.max() > 0 else np.zeros_like(gray)
        _, mask = cv2.threshold(mag, 100, 255, cv2.THRESH_BINARY)
        # Dilate to fill gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.dilate(mask, kernel, iterations=1)
        return mask

    def _detect_combined_vote(self, bgr):
        """Multi-method voting: brightness + HSV + LAB. Pixel is white if ≥2 agree."""
        m1 = self._detect_brightness(bgr)
        m2 = self._detect_hsv(bgr)
        m3 = self._detect_lab(bgr)
        # Vote: each mask contributes 1, need ≥2
        vote = (m1.astype(np.int16) + m2.astype(np.int16) + m3.astype(np.int16))
        mask = np.where(vote >= 2 * 255, 255, 0).astype(np.uint8)
        return mask

    def _detect_morpho_skeleton(self, bgr):
        """Brightness threshold + morphological skeleton to thin lines."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        thresh = self._p('brightness_thresh')
        _, binary = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)
        binary = self._morph_clean(binary)

        # Morphological skeleton (Zhang-Suen thinning)
        skeleton = np.zeros_like(binary)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        done = False
        img = binary.copy()
        while not done:
            eroded = cv2.erode(img, element)
            dilated = cv2.dilate(eroded, element)
            diff = cv2.subtract(img, dilated)
            skeleton = cv2.bitwise_or(skeleton, diff)
            img = eroded.copy()
            if cv2.countNonZero(img) == 0:
                done = True

        # Dilate skeleton slightly for point coverage
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        skeleton = cv2.dilate(skeleton, kernel, iterations=2)
        return skeleton

    def _detect_clahe(self, bgr):
        """CLAHE contrast enhancement + brightness threshold."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        clip = self._p('clahe_clip')
        clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        thresh = self._p('brightness_thresh')
        _, mask = cv2.threshold(enhanced, thresh, 255, cv2.THRESH_BINARY)
        return self._morph_clean(mask)


def main(args=None):
    rclpy.init(args=args)
    node = LaneCandidateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
