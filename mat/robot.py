class Robot:
    myCoord = ()
    alive = False
    myID = 0
    myPath = []
    
    def __init__(self, mID, xyCoord):
        self.myCoord = xyCoord
        self.myID = mID
        self.myPath.append(xyCoord)

        if mID == 0:
            self.alive = True
    
    def goto(self, xyCoord):
        self.myPath.append(xyCoord)
        self.myCoord = xyCoord
        
        
    
        
    
        
        
    
