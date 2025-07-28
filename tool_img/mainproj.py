from PIL import Image, ImageDraw, ImageFont

#
# function add text for image
#
def add_text_to_image(iamge_path, text, output_path, 
                    font_path=None, font_size=40,
                    text_color=(255, 255, 255), position=(10, 10)):
    try:
        img =Image.open(iamge_path)
        draw = ImageDraw.Draw(img)
        if font_path:
            try:
                font = ImageFont.truetype(font_path, font_size)
            except IOError:
                print(f"Not found path font: {font_path}. Use default font.")
                font = ImageFont.load_default()
        else:
            font = ImageFont.load_default()

        draw.text(position, text, font=font, fill=text_color)

        img.save(output_path)

        print(f"Success")

    except FileNotFoundError:
        print(f"Error: Not found current path image {iamge_path}")
    except Exception as e:
        print(f"Error code : {e}")


if __name__ == "__main__":
    try:
        dummy_img = Image.new('RGB', (600, 400), color = 'lightblue')
        draw_dummy = ImageDraw.Draw(dummy_img)
        draw_dummy.text((200, 180), "Image Demo", fill=(0,0,0), font=ImageFont.load_default())
        dummy_img.save("image/my_image.png")
        print("Created image demo : my_image.png")
    except Exception as e:
        print(f"Can't create demo image : {e}. Please make sure extist your image.")    

    add_text_to_image(
        iamge_path="image/2.png",
        text="Nguyen Van A",
        output_path="image_out/Nguyen_van_a.png",
        position=(1088, 189),
        text_color=(255, 0, 0) # Red
    )    

