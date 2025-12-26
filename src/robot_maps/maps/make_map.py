import numpy as np
import cv2

# Map size (pixels)
width, height = 200, 200   # 200x200 pixels

# Start with all free space (white = 255)
map_img = 255 * np.ones((height, width), dtype=np.uint8)

# Draw outer walls (black = 0)
cv2.rectangle(map_img, (0,0), (width-1, height-1), 0, thickness=5)

# Add obstacles (black shapes)
cv2.rectangle(map_img, (50,50), (80,120), 0, -1)   # block obstacle
cv2.rectangle(map_img, (120,40), (160,60), 0, -1)  # small wall
cv2.circle(map_img, (140,140), 20, 0, -1)          # circular obstacle

# Save as PNG
cv2.imwrite("simple_map.png", map_img)
print("✅ Map saved as simple_map.png")
