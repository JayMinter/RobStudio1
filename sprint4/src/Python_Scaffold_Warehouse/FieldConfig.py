class FieldConfig:
    def __init__(self, name, crop, fieldSize, rows, cropsNum):
        self.name = name
        self.crop = crop
        self.fieldSize = fieldSize
        self.rows = rows
        self.cropsNum = cropsNum

    def getName(self):
        return self.name
    
    def getCrop(self):
        return self.crop
    
    def getRows(self):
        return self.rows
    
    def getNumCrops(self):
        return self.cropsNum
    
    def getXFieldSize(self):
        return self.fieldSize[0]
    
    def getYFieldSize(self):
        return self.fieldSize[1]