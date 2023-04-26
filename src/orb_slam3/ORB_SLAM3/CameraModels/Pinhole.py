from ORB_SLAM3.CameraModels.GeometricCamera import GeometricCamera

class Pinhole(GeometricCamera):
    # def serial()
    def __init__(self, _vParameters=None, pPinhole=None):
        self.tvr = None
        self.mvParameters = [None] * 4
        self.mnId = self.nNextId + 1
        self.nNextId += 1
        self.mnType = self.CAM_PINHOLE

        if _vParameters != None:
            assert len(_vParameters) == 4, "len(_vParameters) != 4"
            self.mvParameters = _vParameters
            return
        if pPinhole != None:
            assert len(pPinhole.mvParameters) == 4, "len(pPinhole.mvParameters) != 4"
            self.mvParameters = pPinhole.mvParameters
            return
    
