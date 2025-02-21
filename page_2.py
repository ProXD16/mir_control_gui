import tkinter as tk

class Page2(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        tk.Label(self, text="Trang 2 - Cảm biến", font=("Arial", 14)).pack(pady=20)
