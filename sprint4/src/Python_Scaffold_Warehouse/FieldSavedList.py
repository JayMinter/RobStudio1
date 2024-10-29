from FieldConfig import FieldConfig

class FieldSavedList:
    def __init__(self):
        self.saved = []

    def saveConfig(self, name, crop, fieldSize, rows, cropNum):
        # print("number of saved configs: ", len(self.saved))
        self.saved.append(FieldConfig(name, crop, fieldSize, rows, cropNum))

    def getAllConfigs(self):
        return self.saved
    
    def getConfig(self, name):
        if len(self.saved) >= 0:
            for config in self.saved:
                if config.getName() == name:
                    return config