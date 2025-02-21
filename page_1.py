import tkinter as tk

class Page1(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        tk.Label(self, text="Trang 1 - Điều khiển", font=("Arial", 14)).pack(pady=20)

        # Biến cục bộ chạy ngầm
        self.counter = 0
        self.label_counter = tk.Label(self, text="Counter: 0", font=("Arial", 12))
        self.label_counter.pack()
        self.update_counter()

    def update_counter(self):
        self.counter += 1
        self.label_counter.config(text=f"Counter: {self.counter}")
        self.after(1000, self.update_counter)
