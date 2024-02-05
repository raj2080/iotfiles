
from PIL import Image

import pytesseract




image_path = 'D:/numnber_plate/plates/scaned_img_0.jpg'
img = Image.open(image_path)
print("Image opened successfully.")


text = pytesseract.image_to_string(img, lang="eng")
print("Text extracted:", text)


