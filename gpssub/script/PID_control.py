import math

def SetTrack():
    Track = [[1,1] , [2,2] , [3,3]]  ###2_D tuple
    return Track

def GetTrackError(LocationX , LocationY , Heading):
    Target = []
    Track = SetTrack()

    for i in range(len(Track)):
        dx = Track[i][0] - LocationX
        if dx >= 0:
            if len(Track) - i >= 10:###to be decided
                Target = Track[i+10]##to be decided
            else:
                Target = Track[-1]
        Dx = Target[0] - LocationX
        Dy = Target[1] - LocationY
        DHeading = math.atan(Dx , Dy)
    return Dx , Dy , DHeading

def Control(Dx , Dy , DHeading):
    ###Strategy of Speed control and Steering control
    TargetSpeed = 0
    TargetAngle = 0
    return TargetSpeed , TargetAngle

def PID(P , I , D , Target , Output):
    LastError = 0
    LastLastError = 0
    Error = Target - Output

    IncrementValue = (P + I + D) * Error + (P + 2 * D) * LastError + D * LastLastError

    Output += IncrementValue
    LastLastError = LastError
    LastError = Error
    return Output

def SpeedControl(TargetSpeed , PresentSpeed , P , I , D):
    # P = 0.58
    # I = 0.3
    # D = 0.1
    SpeedOutput = PID(P , I , D , TargetSpeed , PresentSpeed)

    return SpeedOutput

def SteeringControl(TargetAngle , PresentAngle , P , I , D):
    # P = 3.5
    # I = 0.8
    # D = 0.1
    SteeringOutput = PID(P , I , D , TargetAngle , PresentAngle)

    return SteeringOutput

def SignalOutput(SpeedOutput , SteeringOutput):
    SpeedSingal , SteeringSingal = 0 , 0
    return SpeedSingal , SteeringSingal

def main(LocationX , LocationY , Heading , Ve , Vn , P , I , D):
    PresentSpeed = pow((Ve**2 + Vn**2) , 0.5) #PresentSpeed is a function of Ve and Vn
    PresentAngle = 0 #PresentAngle is feedback of sensor
    ErrorX , ErrorY , ErrorHeading = GetTrackError(LocationX , LocationY , Heading)
    TargetSpeed , TargetAngle = Control(ErrorX , ErrorY , ErrorHeading)
    SpeedOutput = SpeedControl(TargetSpeed , PresentSpeed , P , I , D)
    SteeringOutput = SteeringControl(TargetAngle , PresentAngle , P , I , D)
    CommandSpeed , CommandSteer = SignalOutput(SpeedOutput , SteeringOutput)

    return CommandSpeed , CommandSteer
