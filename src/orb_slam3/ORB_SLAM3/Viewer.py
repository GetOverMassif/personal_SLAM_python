import cv2
import sys

from Tools import errorPrint

class Viewer:
    def __init__(self, pSystem, pFrameDrawer, pMapDrawer, pTracking, strSettingPath, settings):
        self.both = False
        self.mpSystem = pSystem
        self.mpFrameDrawer = pFrameDrawer
        self.mpMapDrawer = pMapDrawer
        self.mpTracker = pTracking
        self.mbFinishRequested = False
        self.mbFinished = True
        self.mbStopped = True
        self.mbStopRequested = False

        if self.settings:
            self.newParameterLoader(settings)
        else:

            fSettings = cv2.FileStorage(strSettingPath, cv2.FileStorage_READ);

            is_correct = self.ParseViewerParamFile(fSettings)

            if not is_correct:
                errorPrint("**ERROR in the config file, the format is not correct**\n")
                # try:
                sys.exit(-1)
                # catch(exception &e)


        self.mbStopTrack = False
    