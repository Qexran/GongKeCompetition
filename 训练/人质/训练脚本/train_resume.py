# -*- coding: utf-8 -*-
"""YOLOv8 resume training script."""
from ultralytics import YOLO

# Load last checkpoint to resume training
model = YOLO('新/last.pt')

# Resume training
results = model.train(
    data='dataset/data.yaml',
    epochs=150,
    imgsz=1280,
    batch=4,
    patience=30,
    optimizer='AdamW',
    lr0=0.001,
    lrf=0.01,
    device='cpu',
    project='新',
    name='train',
    exist_ok=True,
    pretrained=True,
    augment=True,
    mosaic=1.0,
    mixup=0.1,
    copy_paste=0.1,
    workers=4,
    verbose=True,
    resume=True,
)

print("Training complete!")
print(f"Best model saved to: 新/train/weights/best.pt")
