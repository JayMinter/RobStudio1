from tkinter import *
from tkinter import ttk
# from PIL import Image, ImageTk

class Utils:

    darkestGreen = "#385623"
    midGreen= "#538135"
    lightGreen = "e2efd9"
    width_ = 600
    image_height = 150

    # def Button(root, text_):
    #     return Button(root, text=text_, background="#168FC1", padx=0, relief=FLAT, font="Arial 11 bold", foreground="white")
    def disable():
        pass

    def Toplevel(title_):
        tl = Toplevel()
        tl.resizable(False, False)
        tl.title(title_)
        tl.protocol("WM_DELETE_WINDOW", Utils.disable)
        return tl

    def Button(root, text_, callback=None):
        return Button(root, text=text_, command=callback, background=Utils.midGreen, padx=10, relief=FLAT, font="Arial 11 bold", foreground="white")
    
    def Frame(root):
        return Frame(root, width=Utils.width_)
    
    def Separator(root):
        return ttk.Separator(root, orient='horizontal')

    def Label(root, text_):
        return Label(root, text=text_, font="Helvetica 12 bold", foreground=Utils.darkestGreen)

    
    #Dont use this for the icon image
    def Image(root, path):
        image_ = ImageTk.PhotoImage(Image.open(path).resize((Utils.width_, Utils.image_height)))
        lbl = Label(root, image=image_)
        lbl.photo = image_
        return lbl