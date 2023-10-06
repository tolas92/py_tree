import tkinter as tk
import random

class BlinkingEyes:
    def __init__(self, root):
        self.root = root
        self.root.title("Blinking Eyes and Mouth")
        self.canvas = tk.Canvas(root, width=root.winfo_screenwidth(), height=root.winfo_screenheight(), bg="black")
        self.canvas.pack()
        self.eye_color = "teal"
        self.mouth_color = "teal"
        self.blinking = False
        self.draw_eyes()
        self.draw_mouth()
        self.animate()

    def draw_eyes(self):
        self.canvas.delete("eyes")
        #eye_size = min(self.root.winfo_screenwidth(), self.root.winfo_screenheight()) // 10
        eye_width = min(self.root.winfo_screenwidth(), self.root.winfo_screenheight()) // 20  # Decreased eye width
        eye_height = min(self.root.winfo_screenwidth(), self.root.winfo_screenheight()) // 10  # Eye height remains the same
        eye_x_offset = self.root.winfo_screenwidth() // 4
        eye_y_offset = self.root.winfo_screenheight() // 3

        self.canvas.create_rectangle(eye_x_offset - eye_width, eye_y_offset - eye_height,
                                     eye_x_offset + eye_width, eye_y_offset + eye_height,
                                     fill=self.eye_color, tags="eyes")

        self.canvas.create_rectangle(3 * eye_x_offset - eye_width, eye_y_offset - eye_height,
                                     3 * eye_x_offset + eye_width, eye_y_offset + eye_height,
                                     fill=self.eye_color, tags="eyes")
    """
    def draw_mouth(self):
        mouth_width = min(self.root.winfo_screenwidth(), self.root.winfo_screenheight()) // 8
        mouth_height = mouth_width // 3
        mouth_x_offset = self.root.winfo_screenwidth() // 2
        mouth_y_offset = 3 * self.root.winfo_screenheight() // 4

        self.canvas.create_rectangle(mouth_x_offset - mouth_width, mouth_y_offset - mouth_height,
                                     mouth_x_offset + mouth_width, mouth_y_offset + mouth_height,
                                     fill=self.mouth_color, tags="mouth")
    """
    def draw_mouth(self):
        mouth_width = min(self.root.winfo_screenwidth(), self.root.winfo_screenheight()) // 8
        mouth_height = mouth_width // 3
        mouth_x_offset = self.root.winfo_screenwidth() // 2
        mouth_y_offset = 3 * self.root.winfo_screenheight() // 4

    # Draw the mouth
        self.canvas.create_rectangle(mouth_x_offset - mouth_width, mouth_y_offset - mouth_height,
                                 mouth_x_offset + mouth_width, mouth_y_offset + mouth_height,
                                 fill=self.mouth_color, outline=self.mouth_color, tags="mouth")

    # Draw small rectangles on either side of the mouth
        side_rect_width = mouth_width // 4
        side_rect_height = mouth_height # Slightly higher than mouth
        left_side_rect_x = mouth_x_offset - mouth_width - side_rect_width
        right_side_rect_x = mouth_x_offset + mouth_width
        side_rect_y = mouth_y_offset - mouth_height // 1.5

    # Draw left side rectangle
        self.canvas.create_rectangle(left_side_rect_x, side_rect_y,
                                 left_side_rect_x + side_rect_width, side_rect_y + side_rect_height,
                                 fill=self.mouth_color, outline=self.mouth_color, tags="mouth")

    # Draw right side rectangle
        self.canvas.create_rectangle(right_side_rect_x, side_rect_y,
                                 right_side_rect_x + side_rect_width, side_rect_y + side_rect_height,
                                 fill=self.mouth_color, outline=self.mouth_color, tags="mouth")

    def blink(self):
        if self.blinking:
            self.eye_color = "black"  # simulate blinking by hiding the eyes
        else:
            self.eye_color = "teal"
        self.blinking = not self.blinking
        self.draw_eyes()

    def animate(self):
        self.blink()
        self.root.after(random.randint(500, 2000), self.animate)  # change blinking speed

if __name__ == "__main__":
    root = tk.Tk()
    eyes = BlinkingEyes(root)
    root.mainloop()
