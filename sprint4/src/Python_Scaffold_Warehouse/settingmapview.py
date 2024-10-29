import tkinter as tk
from tkinter import ttk
from Utils import Utils
import subprocess

class SettingMapView(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)

        self.controller = None
        self.titleLbl = Utils.Label(self, "Agricultural Monitering System")
        self.titleLbl.pack(side=tk.TOP)
        self.windowsFrame = Utils.Frame(self)
        self.windowsFrame.pack(side=tk.TOP)
        self.windowsFrame.columnconfigure(0, weight=1)
        self.startBtn = Utils.Button(self.windowsFrame, "Start", self.openMapSetting)
        # self.startBtn.config(width=300)
        self.startBtn.grid(row=0, column=0, sticky="ew")
        self.reportsBtn = Utils.Button(self.windowsFrame, "View Reports")
        # self.reportsBtn.config(width=300)
        self.reportsBtn.grid(row=0, column=1, sticky="ew")


    def openMapSetting(self):
        vcmd = (self.register(self.validateNum))
        # self.controller = None
        self.ros_process = None

        self.mapSettingWindow = Utils.Toplevel("Crop-Guard")

        self.headerFrame = Utils.Frame(self.mapSettingWindow)
        self.exitBtn=Utils.Button(self.headerFrame, "Exit",  self.exit_button_clicked) #  lambda:self.back()
        self.exitBtn.pack(side=tk.LEFT)
        self.title = Utils.Label(self.headerFrame, "Enter Farm Details")
        self.title.pack(side=tk.TOP)

        self.savedFrame = Utils.Frame(self.mapSettingWindow)
        self.savedLbl = Utils.Label(self.savedFrame, "Presaved Configs")
        self.savedLbl.grid(row=0, column=0)
        # creating the drop down menu for the Configs to be chosen
        self.configsDd = ttk.Combobox(self.savedFrame, width=15)
        self.configsDd.grid(row=0, column=1)
        self.chooseBtn = Utils.Button(self.savedFrame, "Select", lambda:self.configSelect())
        self.chooseBtn.grid(row=0, column=2)

        self.variablesFrame = Utils.Frame(self.mapSettingWindow)
        self.cropLbl = Utils.Label(self.variablesFrame, "Crop Type")
        self.cropLbl.grid(row=0, column=0)
        # creating the drop down menu for the crop type
        options = ["Corn", "Lettuce", "Kale", "Beans"]
        self.cropTypeDd = ttk.Combobox(self.variablesFrame, value=options, width=15)
        self.cropTypeDd.grid(row=0, column=1)

        self.fieldSizeLbl = Utils.Label(self.variablesFrame, "Field Size")
        self.fieldSizeLbl.grid(row=1, column=0)
        # this is the input box for the field size with x and y variables
        self.fieldVarFrame = Utils.Frame(self.variablesFrame)
        self.fieldVarFrame.grid(row=1, column=1)
        self.xSizeTf = tk.Entry(self.fieldVarFrame, width=5,validate='all', validatecommand=(vcmd, '%P'))
        self.xSizeTf.pack(side=tk.LEFT)
        self.set_text(self.xSizeTf, "1")
        self.midLbl = tk.Label(self.fieldVarFrame, text="m X ")
        self.midLbl.pack(side=tk.LEFT)
        self.ySizeTf = tk.Entry(self.fieldVarFrame, width=5, validate='all', validatecommand=(vcmd, '%P'))
        self.ySizeTf.pack(side=tk.LEFT)
        self.set_text(self.ySizeTf, "1")
        self.endLbl = tk.Label(self.fieldVarFrame, text="m")
        self.endLbl.pack(side=tk.LEFT)


        self.rowNumLbl = Utils.Label(self.variablesFrame, "No. of Rows")
        self.rowNumLbl.grid(row=2, column=0)
        # creating the incrementor
        self.rowIncrFrame1 = Utils.Frame(self.variablesFrame)
        self.rowIncrFrame1.grid(row=2, column=1)
        self.rowTf = tk.Entry(self.rowIncrFrame1, width=13, validate='all', validatecommand=(vcmd, '%P'))
        self.set_text(self.rowTf, "1")
        self.decrBtn1 = tk.Button(self.rowIncrFrame1, text="-", command=lambda:self.decrease(self.rowTf))
        self.decrBtn1.pack(side=tk.LEFT)
        self.rowTf.pack(side=tk.LEFT)
        self.incrBtn1 = tk.Button(self.rowIncrFrame1, text="+", command=lambda:self.increase(self.rowTf))
        self.incrBtn1.pack(side=tk.LEFT)

        self.cropNumLbl = Utils.Label(self.variablesFrame, "No. of Crops/Rows")
        self.cropNumLbl.grid(row=3, column=0)
        # incrementor again
        self.rowIncrFrame2 = Utils.Frame(self.variablesFrame)
        self.rowIncrFrame2.grid(row=3, column=1)
        self.cropNumTf = tk.Entry(self.rowIncrFrame2, width=13, validate='all', validatecommand=(vcmd, '%P'))
        self.set_text(self.cropNumTf, "1")
        self.decrBtn2 = tk.Button(self.rowIncrFrame2, text="-", command=lambda:self.decrease(self.cropNumTf))
        self.decrBtn2.pack(side=tk.LEFT)
        self.cropNumTf.pack(side=tk.LEFT)
        self.incrBtn2 = tk.Button(self.rowIncrFrame2, text="+", command=lambda:self.increase(self.cropNumTf))
        self.incrBtn2.pack(side=tk.LEFT)

        self.btnFrame = Utils.Frame(self.mapSettingWindow)
        self.confirmBtn = Utils.Button(self.btnFrame, "Confirm", self.confirm_button_clicked)
        self.confirmBtn.grid(row=0, column=0)
        self.loadBtn = Utils.Button(self.btnFrame, "Load Presaved", self.load_button_clicked)
        self.loadBtn.grid(row=0, column=1)
        self.saveBtn = Utils.Button(self.btnFrame, "Save Config", self.save_button_clicked)
        self.saveBtn.grid(row=0, column=2)
        self.demoBtn = Utils.Button(self.btnFrame, "Demo", self.demo_button_clicked)
        self.demoBtn.grid(row=0, column=3)

        # Packing the frames so they show in the correct order
        self.btnFrame.pack(side=tk.BOTTOM)
        self.variablesFrame.pack(side=tk.BOTTOM)
        self.headerFrame.pack(side=tk.TOP)
    
    def openNamingWindow(self):
        self.namingWindow = Utils.Toplevel(self)
        
        self.nameLbl = Utils.Label(self.namingWindow, "Config Name")
        self.nameLbl.pack(side=tk.LEFT)
        self.nameTf = tk.Entry(self.namingWindow, width=20)
        self.nameTf.pack(side=tk.LEFT)
        self.submitBtn = Utils.Button(self.namingWindow, "Save", self.saveName)
        self.submitBtn.pack(side=tk.LEFT)

    def openROSControllWindow(self):
        self.rosControlWindow = Utils.Toplevel(self)

        self.stopBtn = Utils.Button(self.rosControlWindow, "Stop ROS Node", self.stopRosNode)
        self.stopBtn.pack()

    
    def set_controller(self, controller):
        self.controller = controller

    def set_text(self, tf, text):
        tf.delete(0, tk.END)
        tf.insert(0, text)

    # Entry boxes that use this mean it only allows for numbers to be written in them
    def validateNum(self, P):
        if str.isdigit(P) or P == "":
            return True
        else:
            return False

    def exit_button_clicked(self):
        self.mapSettingWindow.destroy()
    
    def confirm_button_clicked(self):
        if self.controller:
            try:
                self.controller.confirm()
            except:
                pass
    
    def load_button_clicked(self):
        if self.controller and len(self.controller.getSaved()) > 0:
            try:
                configs = self.controller.getSaved()
                options = []
                for config in configs:
                    options.append(config.getName())
                self.configsDd.config(value=options)
                self.savedFrame.pack(side=tk.TOP)                
            except:
                pass
    
    def configSelect(self):
        try:
            if self.controller:
                configName = self.configsDd.get()
                print(configName)
                if self.controller.config(configName) != None:
                    print("Hello, I am here")
                    config = self.controller.config(configName)
                    self.cropTypeDd.set(config.getCrop())
                    self.set_text(self.xSizeTf, str(config.getXFieldSize()))
                    self.set_text(self.ySizeTf, str(config.getYFieldSize()))
                    self.set_text(self.rowTf, str(config.getRows()))
                    self.set_text(self.cropNumTf, str(config.getNumCrops()))
        except:
            print("The try failed?")
            pass


    def save_button_clicked(self):
        if self.controller:
            try:
                self.openNamingWindow()
            except:
                pass
    
    def saveName(self):
        try:
            if self.nameTf.get() != None:
                name = self.nameTf.get()
                crop = self.cropTypeDd.get()
                fieldSize = [int(self.xSizeTf.get()), int(self.ySizeTf.get())]
                rows = int(self.rowTf.get())
                cropNum = int(self.cropNumTf.get())
                self.controller.save(name, crop, fieldSize, rows, cropNum)
                self.namingWindow.destroy()
        except:
            pass

    def demo_button_clicked(self):
        if self.controller and self.ros_process is None:
            try:
                self.openROSControllWindow()
                self.mapSettingWindow.destroy()
                self.ros_process = subprocess.Popen(["ros2", "launch", "sprint4", "cyl.launch.py"])
                print("Ros Node should have started")

            except Exception as e:
                print(f"Failed to start ROS2 Launch: {e}")

    def stopRosNode(self):
        if self.ros_process is not None:
            self.ros_process.terminate()
            self.ros_process = None
            print("Ros Node should have stopped")

    def increase(self, tf):
        # print("increase function called")
        num = int(tf.get())
        if num >= 1:
            num += 1
            self.set_text(tf, str(num)) 

    def decrease(self, tf):
        # print("decrease function called")
        num = int(tf.get())
        if num > 1:
            num -= 1
            self.set_text(tf, str(num)) 
