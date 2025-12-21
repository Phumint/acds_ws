import cv2 as cv
import numpy as np

class SearchBox():
    def __init__(self, frame, mask, lx=80, rx=150, y=245, width=100, height=20, num_boxes=10):
        """
        Initialize the detector with a mask and ROI parameters.
        
        Parameters:
        - mask: Binary mask (numpy array)
        - lx, rx: Initial x positions for left and right lanes
        - y: Bottom starting y coordinate
        - width, height: Dimensions of rectangle
        - num_boxes: Number of boxes to stack
        """
        self.mask = mask
        self.frame = frame
        self.y = y
        self.width = width
        self.height = height
        self.num_boxes = num_boxes
        
        # Store initial positions for reset
        self.initial_lx = lx
        self.initial_rx = rx

        # Calculate center line
        self.center_x = mask.shape[1] // 2

        # Store independent positions for each box
        self.left_positions = [lx] * num_boxes
        self.right_positions = [rx] * num_boxes
        
        self.roi_mask = None
        self.avg_x = None
    
    def set_roi(self, x, y, width, height):
        """
        Change the location and size of the ROI rectangle.
        
        Parameters:
        - x, y: New top-left corner coordinates
        - width, height: New dimensions
        """
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def detect(self, x, y):
        """
        Detect and count pixels in the current ROI.
        Returns the new x position or None if no detection.
        """
        # Ensure coordinates are within bounds
        x = max(0, x)
        y = max(0, y)
        width = min(self.width, self.mask.shape[1] - x)
        height = min(self.height, self.mask.shape[0] - y)
        
        # Extract ROI from mask
        self.roi_mask = self.mask[y:y+height, x:x+width]

        # If ROI is 3-channel, convert to gray
        if self.roi_mask.ndim == 3:
            gray = cv.cvtColor(self.roi_mask, cv.COLOR_BGR2GRAY)
        else:
            gray = self.roi_mask

        # Find non-zero pixels and compute average x in global coords
        ys, xs = np.nonzero(gray)
        if xs.size > 0:
            avg_x = x + xs.mean()
            # Recenter: left edge so avg_x sits at box center
            new_x = int(avg_x - self.width // 2)
            # Allow box to leave screen bounds
            return new_x
        else:
            # No pixels found - return None to signal no detection
            return None

    def visualize(self):
        """
        Create a visualization of the mask with ROI highlighted.
        Box 0 (first/bottom) stays fixed. Other boxes follow detection or interpolate.
        """
        vis = self.frame.copy() if self.frame.ndim == 3 else cv.cvtColor(self.frame, cv.COLOR_GRAY2BGR)
        rlane = ([], [])
        llane = ([], [])

        # First pass: detect all boxes
        left_detections = []
        right_detections = []
        
        for i in range(self.num_boxes):
            box_y = self.y - i * (self.height + 5)
            box_y = max(0, min(box_y, self.mask.shape[0] - self.height))
            
            # Detect for left lane
            left_new_x = self.detect(self.left_positions[i], box_y)
            left_detections.append(left_new_x)
            
            # Detect for right lane
            right_new_x = self.detect(self.right_positions[i], box_y)
            right_detections.append(right_new_x)

        # Check if all boxes have no detection - if so, reset positions
        if all(d is None for d in left_detections):
            self.left_positions = [self.initial_lx] * self.num_boxes
            left_detections = [self.initial_lx] * self.num_boxes
            
        if all(d is None for d in right_detections):
            self.right_positions = [self.initial_rx] * self.num_boxes
            right_detections = [self.initial_rx] * self.num_boxes

        # Process left lane boxes
        for i in range(self.num_boxes):
            if i == 0:
                # First box (bottom) never moves
                pass
            else:
                new_x = left_detections[i]
                
                if new_x is not None:
                    # Check if box center would cross center line
                    box_center = new_x + self.width // 2
                    if box_center < self.center_x:
                        # Valid detection and doesn't cross
                        self.left_positions[i] = new_x
                    else:
                        # Detection crosses center line - clamp it
                        self.left_positions[i] = self.center_x - self.width // 2
                else:
                    # No detection - interpolate from boxes above and below
                    # Find nearest detected boxes above and below
                    above_idx = None
                    below_idx = None
                    
                    for j in range(i + 1, self.num_boxes):
                        if left_detections[j] is not None:
                            above_idx = j
                            break
                    
                    for j in range(i - 1, -1, -1):
                        if left_detections[j] is not None:
                            below_idx = j
                            break
                    
                    if above_idx is not None and below_idx is not None:
                        # Interpolate between above and below
                        above_x = self.left_positions[above_idx] + self.width // 2
                        below_x = self.left_positions[below_idx] + self.width // 2
                        weight = (i - below_idx) / (above_idx - below_idx)
                        interpolated_center = below_x + weight * (above_x - below_x)
                        interpolated_x = int(interpolated_center - self.width // 2)
                        # Clamp to not cross center
                        interpolated_x = min(interpolated_x, self.center_x - self.width // 2)
                        self.left_positions[i] = interpolated_x
                    elif above_idx is not None:
                        # Only have box above
                        self.left_positions[i] = self.left_positions[above_idx]
                    elif below_idx is not None:
                        # Only have box below
                        self.left_positions[i] = self.left_positions[below_idx]
            
            box_y = self.y - i * (self.height + 5)
            box_y = max(0, min(box_y, self.mask.shape[0] - self.height))
            box_x = self.left_positions[i]

            # Draw rectangle - clamp visualization to stay within frame
            vis_x = max(0, box_x)
            vis_width = min(self.width, self.mask.shape[1] - vis_x)
            if vis_width > 0:
                cv.rectangle(vis, (vis_x, box_y), (vis_x + vis_width, box_y + self.height),
                            (0, 255, 0), 1)
            
            # Center marker - only draw if within bounds
            center_x = box_x + self.width // 2
            center_y = box_y + self.height // 2
            if 0 <= center_x < self.mask.shape[1]:
                cv.circle(vis, (center_x, center_y), 3, (0, 0, 255), -1)
                llane[0].append(center_x)
                llane[1].append(center_y)

        # Process right lane boxes
        for i in range(self.num_boxes):
            if i == 0:
                # First box (bottom) never moves
                pass
            else:
                new_x = right_detections[i]
                
                if new_x is not None:
                    # Check if box center would cross center line
                    box_center = new_x + self.width // 2
                    if box_center > self.center_x:
                        # Valid detection and doesn't cross
                        self.right_positions[i] = new_x
                    else:
                        # Detection crosses center line - clamp it
                        self.right_positions[i] = self.center_x - self.width // 2
                else:
                    # No detection - interpolate from boxes above and below
                    above_idx = None
                    below_idx = None
                    
                    for j in range(i + 1, self.num_boxes):
                        if right_detections[j] is not None:
                            above_idx = j
                            break
                    
                    for j in range(i - 1, -1, -1):
                        if right_detections[j] is not None:
                            below_idx = j
                            break
                    
                    if above_idx is not None and below_idx is not None:
                        # Interpolate between above and below
                        above_x = self.right_positions[above_idx] + self.width // 2
                        below_x = self.right_positions[below_idx] + self.width // 2
                        weight = (i - below_idx) / (above_idx - below_idx)
                        interpolated_center = below_x + weight * (above_x - below_x)
                        interpolated_x = int(interpolated_center - self.width // 2)
                        # Clamp to not cross center
                        interpolated_x = max(interpolated_x, self.center_x - self.width // 2)
                        self.right_positions[i] = interpolated_x
                    elif above_idx is not None:
                        # Only have box above
                        self.right_positions[i] = self.right_positions[above_idx]
                    elif below_idx is not None:
                        # Only have box below
                        self.right_positions[i] = self.right_positions[below_idx]
            
            box_y = self.y - i * (self.height + 5)
            box_y = max(0, min(box_y, self.mask.shape[0] - self.height))
            box_x = self.right_positions[i]

            # Draw rectangle - clamp visualization to stay within frame
            vis_x = max(0, box_x)
            vis_width = min(self.width, self.mask.shape[1] - vis_x)
            if vis_width > 0:
                cv.rectangle(vis, (vis_x, box_y), (vis_x + vis_width, box_y + self.height),
                            (0, 255, 0), 1)
            
            # Center marker - only draw if within bounds
            center_x = box_x + self.width // 2
            center_y = box_y + self.height // 2
            if 0 <= center_x < self.mask.shape[1]:
                cv.circle(vis, (center_x, center_y), 3, (0, 0, 255), -1)
                rlane[0].append(center_x)
                rlane[1].append(center_y)
            
        return vis, llane, rlane