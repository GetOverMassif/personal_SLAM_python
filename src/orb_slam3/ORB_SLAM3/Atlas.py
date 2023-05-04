from ORB_SLAM3.Map import Map

class Atlas:
    def __init__(self, initKFid) -> None:
        self.mnLastInitKFidMap = initKFid
        self.mHasViewer = False
        self.mpCurrentMap = Map()
        self.CreateNewMap()
        pass

    def CreateNewMap(self):
        pass

    def SetInertialSensor(self):
        pass