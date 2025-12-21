import cv2

class inversePerspectiveTransform():
    """Class for performing inverse perspective transformation."""
    def __init__(self, frame):
        self.frame = frame

    def inverse_perspective_transform(self, src_points, dst_points, w, h):
        M = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_frame = cv2.warpPerspective(self.frame, M, (w, h))
        return warped_frame 