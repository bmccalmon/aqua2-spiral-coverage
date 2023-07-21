#!/usr/bin/env python3
import cv2
import os

sand_path = "uncropped/test/sand"
rock_path = "uncropped/test/rock"

save_sand = "test/sand"
save_rock = "test/rock"

chunk_n = 0

def count_files(path):
    print(f"Counting the number of files in the path '{path}'...")
    count = 0
    fd = os.listdir(path)
    for i in fd:
        ip = os.path.join(path, i)
        if os.path.isfile(ip):
            count += 1
    return count

def save_chunks(image, chunk_size, output_folder):
    global chunk_n
    height, width, _ = image.shape
    num_rows = height // chunk_size
    num_cols = width // chunk_size

    for row in range(num_rows):
        for col in range(num_cols):
            y_start = row * chunk_size
            y_end = y_start + chunk_size
            x_start = col * chunk_size
            x_end = x_start + chunk_size

            sub_image = image[y_start:y_end, x_start:x_end]

            # see if sub_image is invalid or contains and white pixels
            check_wp = cv2.cvtColor(sub_image, cv2.COLOR_BGR2GRAY)
            if 255 in check_wp or not (check_wp.shape[0] == chunk_size and check_wp.shape[1] == chunk_size):
                continue

            chunk_n += 1            
            filename = f"{chunk_n}.png"
            output_path = os.path.join(output_folder, filename)
            cv2.imwrite(output_path, sub_image)

def main():
    global sand_path, rock_path, save_sand, save_rock, chunk_n
    sand_img_count = count_files(sand_path)
    rock_img_count = count_files(rock_path)

    # sand
    for image_i in range(sand_img_count):
        print(f"Saving chunks from {sand_path}/{image_i+1}.jpg to {save_sand}...")
        img = cv2.imread(f"{sand_path}/{image_i+1}.jpg")
        save_chunks(img, 20, save_sand)
    chunk_n = 0

    # rock
    for image_i in range(rock_img_count):
        print(f"Saving chunks from {rock_path}/{image_i+1}.jpg to {save_rock}...")
        img = cv2.imread(f"{rock_path}/{image_i+1}.jpg")
        save_chunks(img, 20, save_rock)

if __name__ == "__main__":
    main()
