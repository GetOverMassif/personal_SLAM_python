import os

class ORBVocabulary:
    def __init__(self) -> None:
        pass
    
    def loadFromTextFile(self, strVocFile):
        if os.path.isfile(strVocFile):
            print("File exists")
            return True
        else:
            print("File not")
            return False