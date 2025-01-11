import io
from google.cloud import vision
from PIL import Image as PILImage


def find_center(image_path, object):
    # Initialize the Vision client
    client = vision.ImageAnnotatorClient()

    # Read image into memory
    with io.open(image_path, 'rb') as image_file:
        content = image_file.read()
    image = vision.Image(content=content)

    # Send the image to the API for object localization
    response = client.object_localization(image=image)

    # Extract localized object annotations
    objects = response.localized_object_annotations

    # We’ll look for the object named "Apple" (case-insensitive)
    for obj in objects:
        if obj.name.lower() == object:
            # We have found an "Apple"
            bounding_poly = obj.bounding_poly

            # bounding_poly contains normalized vertices (0 to 1)
            normalized_vertices = bounding_poly.normalized_vertices

            # Calculate the average of the vertices to find the normalized center
            avg_x = sum([v.x for v in normalized_vertices]) / len(normalized_vertices)
            avg_y = sum([v.y for v in normalized_vertices]) / len(normalized_vertices)

            # Convert normalized coordinates back to pixel coordinates
            # 1) Open the image again (e.g., with Pillow) to get width & height
            
            pil_image = PILImage.open(image_path)
            width, height = pil_image.size

            # 2) Multiply normalized coords by actual dimensions
            pixel_x = int(avg_x * width)
            pixel_y = int(avg_y * height)

            print(f"Found with approximate center: ({pixel_x}, {pixel_y})")
            return (pixel_x, pixel_y)

    print("No object detected in the image.")
    return None


if __name__ == "__main__":
    image_path = "image1.jpg"
    center_coordinates = find_center(image_path,'pineapple')
    if center_coordinates:
        print("Center in pixel coordinates:", center_coordinates)
