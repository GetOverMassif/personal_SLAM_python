from ORB_SLAM3.CameraModels.GeometricCamera import GeometricCamera

class KannalaBrandt8(GeometricCamera):
    # def serial()
    def __init__(self, _vParameters=None, pKannala=None):
        self.precision = 1e-6
        self.mvLappingArea = 2.0
        self.tvr = None

        self.mvParameters = [None] * 8
        self.mnId = self.nNextId + 1
        self.nNextId += 1
        self.mnType = self.CAM_FISHEYE

        if _vParameters != None:
            assert len(_vParameters) == 8, "len(_vParameters) != 4"
            self.mvParameters = _vParameters
            return
        if pKannala != None:
            assert len(pKannala.mvParameters) == 8, "len(pPinhole.mvParameters) != 4"
            self.mvParameters = pKannala.mvParameters
            return