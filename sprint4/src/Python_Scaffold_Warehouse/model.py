from FieldSavedList import FieldSavedList

class Model:
    def __init__(self):
        self.savedList = FieldSavedList()
    
    def save(self, name, crop, fieldSize, rows, cropNum):
        self.savedList.saveConfig(name, crop, fieldSize, rows, cropNum)

    def getAllConfigs(self):
        return self.savedList.getAllConfigs()
    
    def getConfig(self, name):
        # print("Made it to model getConfig")
        return self.savedList.getConfig(name)
        

    