# import os

# # Path to the images directory
# image_dir = 'paparazzi-Group6/dronet_training/training/HMB_1/images'
# # image_dir = 'paparazzi-Group6/dronet_training/training/HMB_1/disparities'
# # image_dir = 'paparazzi-Group6/dronet_training/validation/HMB_2/images'
# # image_dir = 'paparazzi-Group6/dronet_training/validation/HMB_2/disparities'

# # This for images
# images = sorted([f for f in os.listdir(image_dir) if f.endswith('.jpg')])

# # This for disparities
# # images = sorted([f for f in os.listdir(image_dir) if f.endswith('.png')])

# # Rename files in order to match frame1, frame2, frame3 format
# for i, img in enumerate(images):
#     # This for images
#     new_name = f"frame{i+1}.jpg"

#     # This for disparities
#     # new_name = f"frame{i+1}.png"

#     old_path = os.path.join(image_dir, img)
#     new_path = os.path.join(image_dir, new_name)
    
#     # Rename file
#     os.rename(old_path, new_path)

# print("✅ Renaming complete.")

import os
from pathlib import Path

# Set the root folder where images are located
folder_path = Path("/home/simina/rpg_public_dronet/data_ours/validation/collision/images")

# File extensions to include
image_extensions = [".jpg", ".jpeg", ".png", ".bmp", ".gif"]

# Collect all image files recursively
image_files = [file for file in folder_path.rglob("*")]

# Rename each file with zero-padded numbers
for i, file in enumerate(image_files, start=1):
    new_name = f"{i:04d}{file.suffix.lower()}"
    new_path = file.with_name(new_name)
    try:
        file.rename(new_path)
        print(f"Renamed: {file.name} → {new_name}")
    except Exception as e:
        print(f"Failed to rename {file}: {e}")

