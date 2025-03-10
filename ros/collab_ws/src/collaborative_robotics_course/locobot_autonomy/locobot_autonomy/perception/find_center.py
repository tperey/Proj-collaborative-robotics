# need to access robot mic and interpret the verbal command 
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D
from scipy.integrate import solve_ivp
import cv2
import os
#from google.colab import files
from PIL import Image, ImageDraw
import wave
#from google.cloud import speech_v1p1beta1 as speech
from google.cloud import speech_v1 as speech
#from google.colab import files
from google.cloud import vision
from PIL import Image as PILImage, ImageDraw, ImageFont
import string 
import re 

# json_key_path = r'C:\Users\capam\Documents\stanford\colloborative_robotics\python-447906-51258c347833.json'
# json_key_path = '/home/locobot/Downloads/python-447906-51258c347833.json'
# os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = json_key_path

class VisionObjectDetector:
    def __init__(self):
        """
        Initialize the Vision client once during object creation.
        """
        self.client = vision.ImageAnnotatorClient()

    def find_center(self, image_bytes, object_name):
        """
        Finds the center of an object (e.g., "pineapple") in the provided image bytes.

        :param image_bytes: The raw bytes of the image.
        :param object_name: The target object name to search for (case-insensitive).
        :return: Tuple (pixel_x, pixel_y) of the object's approximate center, or None if not found.
        """
        # gets center of the box, will need to fix to make sure it grabs the center of the object
        # Step 1: Create the Vision Image object from bytes
        #vision_image = vision.Image(content = image_bytes)
        # Step 2: Send the image to the API for object localization
        #send_image = self.client.object_localization(image=vision_image)
        # Step 3: Extract localized object annotations
        #extraction = send_image.localized_object_annotations 
        # Step 4: Search for the specified object. Hint: Objects returns all detected objects
        #objects = extraction[0]
        # Step 5: Once the object is found, determine the position from the bounding box. Hint: obj.bounding_poly returns the bounding box
        #corners = objects.getCenter()  #bounding_poly.normalized_vertices
        # Step 6: Find the center from the corners of the bounding box
        #x_center = sum(corner.x for corner in corners)/len(corners)
        #y_center = sum(corner.y for corner in corners)/len(corners)
        # Step 7: Return the center in pixel coordinates. Hint: The position of the bounding box is normalized so you will need to convert it back into the dimensions of the image
        opencv_img = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        #height, width = opencv_img.shape[:2]
        #x_pixel = x_center * width
        #y_pixel = y_center * height

        # NEW METHOD USING OPENCV
        gray = cv2.cvtColor(opencv_img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        moment = cv2.moments(contours[0])
        x_pixel = int(moment['m10'],moment['m00'])
        y_pixel = int(moment["m01"],moment['m00'])

        return x_pixel, y_pixel

    def annotate_image(self, image_bytes):
        """
        Detects all objects in the image and returns a PIL Image with
        bounding boxes and labels drawn for each detected object.

        :param image_bytes: The raw bytes of the image.
        :return: A PIL Image object annotated with bounding boxes and labels.
        """
        # Step 1: Create the Vision Image object from bytes
        vision_image = vision.Image(content = image_bytes)
        # Step 2: Send the image to the API for object localization
        send_images = self.client.object_localization(image=vision_image)
        # Step 3: Extract localized object annotations
        extraction = send_images.localized_object_annotations 
        # Step 4: Open the image via PIL for drawing
        opencv_img = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        height, width = opencv_img.shape[:2]
        image_pil = Image.fromarray(cv2.cvtColor(opencv_img,cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(image_pil)
        # Step 5: Iterate through all the objects and draw the bounding boxes on the image.
        for objects in extraction: 
            corners = objects.bounding_poly.normalized_vertices
            box = [(width*corner.x, height*corner.y) for corner in corners] 
            draw.polygon(box,outline='red',width=3)
        # Hint: draw.polygon allows you to draw based on pixel coordinates
        #Step 6: Return the annotated image
        return image_pil
