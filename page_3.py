import tkinter as tk

class Page3(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        tk.Label(self, text="Trang 3 - Hệ thống", font=("Arial", 14)).pack(pady=20)
