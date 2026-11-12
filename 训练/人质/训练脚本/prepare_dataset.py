# -*- coding: utf-8 -*-
"""Prepare YOLOv8 dataset from existing train/valid splits."""
import os
import shutil

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGES_DIR = os.path.join(BASE_DIR, "images")
LABELS_DIR = os.path.join(BASE_DIR, "labels")
DATASET_DIR = os.path.join(BASE_DIR, "dataset")


def copy_files(txt_file, split):
    """Copy images and labels based on train/valid txt file."""
    img_dst = os.path.join(DATASET_DIR, "images", split)
    lbl_dst = os.path.join(DATASET_DIR, "labels", split)

    with open(txt_file, "r") as f:
        lines = f.read().strip().split("\n")

    count = 0
    for line in lines:
        line = line.strip()
        if not line:
            continue
        # Extract filename from path like "data/custom/images/cone_00077.jpg"
        filename = os.path.basename(line)
        name = os.path.splitext(filename)[0]

        # Copy image
        src_img = os.path.join(IMAGES_DIR, filename)
        if os.path.exists(src_img):
            shutil.copy2(src_img, os.path.join(img_dst, filename))
            count += 1

        # Copy label
        lbl_file = name + ".txt"
        src_lbl = os.path.join(LABELS_DIR, lbl_file)
        if os.path.exists(src_lbl):
            shutil.copy2(src_lbl, os.path.join(lbl_dst, lbl_file))

    print(f"{split}: copied {count} images")


if __name__ == "__main__":
    copy_files(os.path.join(BASE_DIR, "train.txt"), "train")
    copy_files(os.path.join(BASE_DIR, "valid.txt"), "val")

    # Verify counts
    for split in ["train", "val"]:
        img_count = len(os.listdir(os.path.join(DATASET_DIR, "images", split)))
        lbl_count = len(os.listdir(os.path.join(DATASET_DIR, "labels", split)))
        print(f"{split}: {img_count} images, {lbl_count} labels")
