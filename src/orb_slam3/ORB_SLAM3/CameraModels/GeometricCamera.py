class GeometricCamera:
    CAM_PINHOLE = 0
    CAM_FISHEYE = 1
    nNextId: int = None

    def __init__(self, _vParameters) -> None:
        self.mnId: int = None
        self.mnType: int = None
        self.mvParameters = _vParameters
    
    def getParameter(self, i):
        return self.mvParameters[i]
    
    def setParameter(self, p, i):
        self.mvParameters[i] = p
    
    def size(self):
        return len(self.mvParameters)
    
    def GetId(self):
        return self.mnId

    def GetType(self):
        return self.mnType