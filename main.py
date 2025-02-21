import tkinter as tk
from page_1 import Page1
from page_2 import Page2
from page_3 import Page3

class ControlPanelApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Điều khiển Robot")

        # Thanh taskbar
        self.taskbar = tk.Frame(root, bg="gray", height=40)
        self.taskbar.pack(side="top", fill="x")

        # Container chứa các trang
        self.container = tk.Frame(root)
        self.container.pack(expand=True, fill="both")

        # Tạo và lưu các trang
        self.pages = {
            "Trang 1": Page1(self.container),
            "Trang 2": Page2(self.container),
            "Trang 3": Page3(self.container),
        }

        # Đặt tất cả trang vào container
        for page in self.pages.values():
            page.grid(row=0, column=0, sticky="nsew")

        # Nút trên taskbar
        for name in self.pages:
            btn = tk.Button(self.taskbar, text=name, command=lambda n=name: self.show_page(n))
            btn.pack(side="left", padx=5, pady=5)

        # Hiển thị trang đầu tiên
        self.show_page("Trang 1")

    def show_page(self, name):
        """Chuyển đến trang điều khiển được chọn"""
        self.pages[name].tkraise()

if __name__ == "__main__":
    root = tk.Tk()
    app = ControlPanelApp(root)
    root.mainloop()
