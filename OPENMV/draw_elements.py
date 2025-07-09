import tkinter as tk

def create_drawing():
    root = tk.Tk()
    root.title("数字3和红线")

    canvas = tk.Canvas(root, width=800, height=800, bg="white")
    canvas.pack()

    # 绘制数字3
    canvas.create_text(400, 400, text="3", font=("Arial", 200), fill="black")

    # 绘制一条垂直的红线，加粗
    # 参数: x1, y1, x2, y2, 颜色, 宽度
    canvas.create_line(700, 0, 700, 800, fill="red", width=50)

    root.mainloop()

if __name__ == "__main__":
    create_drawing()
