import cv2 as cv
import numpy as np

class detect_edges:
    def __init__(self, frame):
        self.frame = frame

    def adjust_gamma(self, image, gamma=1.0):
        inv_gamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
        return cv.LUT(image, table)

    def reduce_glare(self, image, clip_limit=3.0, grid_size=(8, 8)):
        lab = cv.cvtColor(image, cv.COLOR_BGR2LAB)
        l, a, b = cv.split(lab)
        clahe = cv.createCLAHE(clipLimit=clip_limit, tileGridSize=grid_size)
        cl = clahe.apply(l)
        return cv.cvtColor(cv.merge((cl, a, b)), cv.COLOR_LAB2BGR)

    def aoi_mask(self):
        h, w = self.frame.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        rect_height = 1000 
        cv.rectangle(mask, (0, h - rect_height), (w, h), 255, thickness=-1)
        return mask
    
    def adjust_contrast_gray(img, alpha=1.5):
        # img should be a grayscale image (uint8)
        new_img = cv.convertScaleAbs(img, alpha=alpha, beta=0)
        return new_img
    
    def clahe_contrast_gray(img, clip_limit=2.0, grid_size=(8,8)):
        clahe = cv.createCLAHE(clipLimit=clip_limit, tileGridSize=grid_size)
        return clahe.apply(img)

    def canny_edge(self, low=18, high=22):


        contrast = cv.convertScaleAbs(self.frame, alpha=0.2)
        gray = cv.cvtColor(contrast, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray, (7, 7), 100)
        canny = cv.Canny(blur, low, high)

        

        # 4. ROI Masking
        aoi = cv.bitwise_and(canny, canny, mask=self.aoi_mask())
        
        # 5. Edge Detection
        return cv.Canny(aoi, low, high)
    