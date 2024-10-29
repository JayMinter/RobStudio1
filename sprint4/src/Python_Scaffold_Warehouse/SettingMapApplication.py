import tkinter as tk
from model import Model
from settingmapview import SettingMapView
from controller import Controller

class SettingMapApplication(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Crop-Guard")

        model = Model()

        view = SettingMapView(self)
        view.grid(row=0, column=0)

        controller = Controller(model, view)

        view.set_controller(controller=controller)


if __name__ == '__main__':
    app = SettingMapApplication()
    app.mainloop()