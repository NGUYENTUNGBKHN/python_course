from PIL import Image, ImageDraw, ImageFont

def add_text_to_image(image_path, text, output_path, font_path=None, font_size=40, text_color=(255, 255, 255), position=(10, 10)):
    """
    Thêm chữ vào ảnh.

    Args:
        image_path (str): Đường dẫn đến file ảnh đầu vào.
        text (str): Văn bản muốn thêm.
        output_path (str): Đường dẫn để lưu ảnh đầu ra.
        font_path (str, optional): Đường dẫn đến file font chữ (.ttf). Mặc định sẽ dùng font mặc định.
        font_size (int, optional): Kích thước chữ. Mặc định là 40.
        text_color (tuple, optional): Màu chữ dưới dạng RGB (ví dụ: (255, 255, 255) là màu trắng).
        position (tuple, optional): Tọa độ (x, y) để đặt chữ. Mặc định là (10, 10).
    """
    try:
        img = Image.open(image_path)
        draw = ImageDraw.Draw(img)

        # Chọn font chữ
        if font_path:
            try:
                font = ImageFont.truetype(font_path, font_size)
            except IOError:
                print(f"Không tìm thấy font tại đường dẫn: {font_path}. Đang sử dụng font mặc định.")
                font = ImageFont.load_default()
        else:
            font = ImageFont.load_default()

        # Thêm chữ vào ảnh
        draw.text(position, text, font=font, fill=text_color)

        # Lưu ảnh đầu ra
        img.save(output_path)
        print(f"Đã lưu ảnh với chữ thành công tại: {output_path}")

    except FileNotFoundError:
        print(f"Lỗi: Không tìm thấy file ảnh tại đường dẫn {image_path}")
    except Exception as e:
        print(f"Đã xảy ra lỗi: {e}")

# --- Ví dụ sử dụng ---
if __name__ == "__main__":
    # Tạo một ảnh dummy để thử nghiệm nếu bạn chưa có
    try:
        dummy_img = Image.new('RGB', (600, 400), color = 'lightblue')
        draw_dummy = ImageDraw.Draw(dummy_img)
        draw_dummy.text((200, 180), "Ảnh Demo", fill=(0,0,0), font=ImageFont.load_default())
        dummy_img.save("my_image.png")
        print("Đã tạo ảnh demo: my_image.png")
    except Exception as e:
        print(f"Không thể tạo ảnh demo: {e}. Vui lòng đảm bảo bạn có ảnh 'my_image.png' để thử nghiệm.")

    # Cách 1: Sử dụng font mặc định
    add_text_to_image(
        image_path="my_image.png",
        text="Chào mừng đến với Osaka!",
        output_path="output_image_default_font.png",
        position=(50, 50),
        text_color=(255, 0, 0) # Đỏ
    )

    # Cách 2: Sử dụng font tùy chỉnh (ví dụ: Arial.ttf, bạn cần có file font này trên máy)
    # Tìm một file font .ttf trên máy tính của bạn (ví dụ: C:\Windows\Fonts\arial.ttf trên Windows, hoặc /System/Library/Fonts/Arial.ttf trên macOS)
    # font_duong_dan = "/System/Library/Fonts/Arial.ttf" # Thay đổi đường dẫn này theo font của bạn
    # add_text_to_image(
    #     image_path="my_image.png",
    #     text="Thành phố năng động!",
    #     output_path="output_image_custom_font.png",
    #     font_path=font_duong_dan,
    #     font_size=60,
    #     text_color=(0, 128, 0), # Xanh lá
    #     position=(100, 200)
    # )