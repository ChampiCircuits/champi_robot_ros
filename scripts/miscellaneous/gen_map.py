from PIL import Image, ImageDraw


Lm = 2 # length in meters
Hm = 3 # height in meters

res = 40 # resolution pxl/m

L = int(Lm * res) + 2
H = int(Hm * res) + 2

print(f"Image size: {L}x{H}")

# Create a new image with white background
img = Image.new('RGB', (L, H), 'white')

# Create a draw object
draw = ImageDraw.Draw(img)

# Draw a black rectangle (contour)
draw.rectangle([(0, 0), (L-1, H-1)], outline='black')

# Save the image as a JPG file
print("Saving image as output.jpg")
img.save('output.jpg')
