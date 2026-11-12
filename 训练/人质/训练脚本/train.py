# -*- coding: utf-8 -*-
"""YOLOv8 continue training script."""
from ultralytics import YOLO

# Load pretrained model
model = YOLO('旧/best.pt')

# Train with optimized parameters for 1080P images
results = model.train(
    data='dataset/data.yaml',
    epochs=150,
    imgsz=1280,        # 1080P images, use larger input size
    batch=4,           # CPU-friendly batch size
    patience=30,       # Early stopping patience
    optimizer='AdamW',
    lr0=0.001,         # Lower LR for fine-tuning
    lrf=0.01,          # Final LR = lr0 * lrf
    device='cpu',      # No GPU available
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
)

print("Training complete!")
print(f"Best model saved to: 新/train/weights/best.pt")
