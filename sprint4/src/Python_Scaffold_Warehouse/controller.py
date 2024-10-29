import subprocess

class Controller:
    def __init__(self, model, view):
        self.model = model
        self.view = view

    def confirm(self):
        print("Confirm button pressed")

    def save(self, name, crop, fieldSize, rows, cropNum):
        # print("Save button pressed")
        try:
            self.model.save(name, crop, fieldSize, rows, cropNum)
        except:
            pass

    

    def getSaved(self):
        return self.model.getAllConfigs()
    
    def config(self, name):
        return self.model.getConfig(name)
        