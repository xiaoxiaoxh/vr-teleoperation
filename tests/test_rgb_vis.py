import os
import cv2

def play_images_from_folder(folder_path, delay=100):
    image_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.jpg')])

    if not image_files:
        print("No JPG images found in the folder.")
        return

    for image_file in image_files:
        image_path = os.path.join(folder_path, image_file)
        image = cv2.imread(image_path)

        if image is None:
            print(f"Could not read image {image_path}. Skipping.")
            continue

        cv2.imshow('Image Sequence', image)

        if cv2.waitKey(delay) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    folder_path = 'data/test/rgb_8_19_17_45'
    play_images_from_folder(folder_path, delay=10) 
