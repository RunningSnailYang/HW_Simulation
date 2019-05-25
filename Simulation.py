# coding=utf-8
import numpy as np
import time

class CarClass(object):
    def __init__(self, Car, TimeAndRoute, CrossToIdx, CrossRoadToNext, CrossAdjacency):
        self.MaxVelocity = Car[3]
        self.Number = Car[0]
        self.StartTime = TimeAndRoute[1]
        self.IfWait = True
        self.Position = 0
        self.Route = self.GetRoute(Car, TimeAndRoute, CrossToIdx, CrossRoadToNext, CrossAdjacency)
        self.RoutePosition = 0
        self.NextRoad = self.Route[0]
        self.StartCross = CrossToIdx[Car[1]]

    # 根据Answer返回行车路线，Road标签为整合后Road列表的Index    
    @staticmethod
    def GetRoute(Car, TimeAndRoute, CrossToIdx, CrossRoadToNext, CrossAdjacency):
        '''
        Car: 车对象
        TimeAndRoute: 车辆的Answer, 类型为Tuple
        CrossToIdx: 从路口编号到路口列表index的字典
        CrossRoadToNext: 字典. 形式为: {(ThisCross, Road): NextRoad, ......}
        CrossAdjacency: 路口邻接矩阵, 指定Cross1和Cross2返回从Cross1到Cross2的Road在Road列表中的Index
        '''
        Route = []
        StartCross = Car[1]
        for Road in TimeAndRoute[2:]:
            # print(TimeAndRoute)
            NextCross = CrossRoadToNext[(StartCross, Road)]
            Route.append(CrossAdjacency[CrossToIdx[StartCross], CrossToIdx[NextCross]])
            StartCross = NextCross
        return Route
            
    # 更新车辆的属性RoutePosition, NextRoad, Position至下一条路径
    def UpdateStateToNextRoad(self, PositionNextRoad):
        '''
        PositionNextRoad: 车辆在下一条路径上的位置
        '''
        self.RoutePosition += 1
        # print('sb', self.RoutePosition, len(self.Route))
        if self.RoutePosition >= len(self.Route) - 1:
            self.NextRoad = -1
        else:
            self.NextRoad = self.Route[self.RoutePosition + 1]
        self.Position = PositionNextRoad

class RoadClass(object):
    def __init__(self, Road):
        self.Length = Road[1]
        self.MaxVelocity = Road[2]
        self.NumChannel = Road[3]
        self.Cars = [[] for _ in range(self.NumChannel)]
        self.WaitChannels = set([i for i in range(self.NumChannel)])
        self.WaitFirstPriority = -1

    # 将路径上的所有车辆设为等待状态, 每次调度正式开始前执行
    def SetAllCarWait(self):
        for Channel in self.Cars:
            for Car in Channel:
                Car.IfWait = True

    # 更新每条路径第一调度优先级车辆所在的Channel号, 如果没有可调度的车辆, 则将其更新为-1
    def UpdateFirstPriority(self):
        if len(self.WaitChannels):
            WaitChannelsList = list(self.WaitChannels)
            WaitChannelsList.sort()
            WaitChannelsMax = [self.Cars[ChannelIdx][-1].Position for ChannelIdx in WaitChannelsList]
            self.WaitFirstPriority = WaitChannelsList[WaitChannelsMax.index(max(WaitChannelsMax))]
        else:
            self.WaitFirstPriority = -1

    # 将一条道路上能更新为终止状态的车辆都更新为终止状态
    def UpdateTerminalStateRoad(self):
        for ChannelIdx, Channel in enumerate(self.Cars):
            self.UpdateTerminalStateChannel(ChannelIdx, Channel)
        # print('sb', self.WaitChannels)
        self.UpdateFirstPriority()

    # 将一条Channel上能更新为终止状态的车辆都更新为终止状态
    def UpdateTerminalStateChannel(self, ChannelIdx, Channel):
        if len(Channel):
            CarEnd = Channel[-1]
            if CarEnd.IfWait:
                MaxVelocity = min(self.MaxVelocity, CarEnd.MaxVelocity)
                if CarEnd.Position + MaxVelocity < self.Length:
                    CarEnd.Position += MaxVelocity
                    CarEnd.IfWait = False
                    if ChannelIdx in self.WaitChannels:
                        self.WaitChannels.remove(ChannelIdx)
                else:
                    self.WaitChannels.add(ChannelIdx)
            elif ChannelIdx in self.WaitChannels:
                self.WaitChannels.remove(ChannelIdx)
                    
            for CarIdx in range(len(Channel) - 2, -1, -1):
                CarEarly = Channel[CarIdx + 1]
                CarLater = Channel[CarIdx]
                    
                if CarLater.IfWait:
                    MaxMoveDistance = CarEarly.Position - CarLater.Position - 1
                    MaxVelocity = min(self.MaxVelocity, CarLater.MaxVelocity)
                    if MaxMoveDistance >= MaxVelocity:
                        CarLater.Position += MaxVelocity
                        CarLater.IfWait = False
                    elif not CarEarly.IfWait:
                        CarLater.Position += MaxMoveDistance
                        CarLater.IfWait = False
        elif ChannelIdx in self.WaitChannels:
            self.WaitChannels.remove(ChannelIdx)

class CrossClass(object):
    def __init__(self, RoadUniList, Cross, CrossToIdx, CrossAdjacency):
        self.CrossNum = Cross[0]
        self.ExitRoads, self.EntranceRoads, self.RoadsToTurnIdx = self.OrganizeCrossCont(RoadUniList,
                                                                  Cross, CrossToIdx, CrossAdjacency)
        self.ExitRoadsNum = len(self.ExitRoads)
        self.EntrancePriority = [3 for _ in self.EntranceRoads]
        self.ExitRoadsWaitSchedule = self.ExitRoads
        self.ExitRoadsWaitScheduleMask = [True for _ in self.ExitRoads]
        self.WaitSchedule = True
        self.UnlimitedGarage = [[] for _ in self.EntranceRoads]

    # 整合路口的相关信息
    @staticmethod 
    def OrganizeCrossCont(RoadUniList, Cross, CrossToIdx, CrossAdjacency):
        '''
        返回值:
        ExitRoads: 路口处可能驶出车辆的道路列表
        EntranceRoads: 路口处可能驶入车辆的道路列表
        RoadsToTurnIdx: 给定路口处车辆驶出的道路在ExitRoads中的idx以及车辆驶入道路在EntranceRoads中的idx, 可以获得转向标签(直行, 左转, 右转)
        '''
        CrossIdx = CrossToIdx[Cross[0]]
        ExitRoads = CrossAdjacency[:, CrossIdx][CrossAdjacency[:, CrossIdx] >= 0].tolist()
        ExitRoads.sort()
        EntranceRoads = CrossAdjacency[CrossIdx][CrossAdjacency[CrossIdx] >= 0].tolist()
        EntranceRoads.sort()
        PositionCrossToTurnIdx = np.array([[-1, 1, 0, 2], [2, -1, 1, 0], 
                                           [0, 2, -1, 1], [1, 0, 2, -1]])
        RoadsToTurnIdx = -np.ones((len(ExitRoads), len(EntranceRoads)), np.int)
        for ExitIdx, ExitRoad in enumerate(ExitRoads):
            for EntranceIdx, EntranceRoad in enumerate(EntranceRoads):
                PositionCrossExit = Cross[1:].index(eval(RoadUniList[ExitRoad][1:]))
                PositionCrossEntrance = Cross[1:].index(eval(RoadUniList[EntranceRoad][1:]))
                RoadsToTurnIdx[ExitIdx, EntranceIdx] = PositionCrossToTurnIdx[PositionCrossExit, \
                                                                              PositionCrossEntrance]
        return ExitRoads, EntranceRoads, RoadsToTurnIdx
        

    def UpdateEntrancePriority(self, RoadsList):
        self.EntrancePriority = [3 for _ in self.EntranceRoads]
        for ExitRoad in self.ExitRoadsWaitSchedule:
            ExitRoadIdx = self.ExitRoads.index(ExitRoad)
            ExitRoadObj = RoadsList[ExitRoad]
            ExitRoadObj.UpdateFirstPriority()
            # print(ExitRoadObj.WaitChannels)
            # print(ExitRoadObj.WaitFirstPriority)
            if ExitRoadObj.WaitFirstPriority != -1:
                EntranceRoad = ExitRoadObj.Cars[ExitRoadObj.WaitFirstPriority][-1].NextRoad
                if EntranceRoad != -1:
                    EntranceRoadIdx = self.EntranceRoads.index(EntranceRoad)
                    self.EntrancePriority[EntranceRoadIdx] = min(self.RoadsToTurnIdx[ExitRoadIdx, EntranceRoadIdx],
                                                                 self.EntrancePriority[EntranceRoadIdx])

    def UpdateExitRoadsWaitSchedule(self, RoadsList):
        self.ExitRoadsWaitSchedule = [Road for Road in self.ExitRoads if RoadsList[Road].WaitFirstPriority != -1]
        self.ExitRoadsWaitScheduleMask = [True for _ in range(len(self.ExitRoadsWaitSchedule))]
        
    def AddCarFromGarage(self, RoadsList):
        for RoadIdx, RoadGarage in enumerate(self.UnlimitedGarage):
            while(len(RoadGarage)):
                CarHasAdd = False
                RoadObj = RoadsList[self.EntranceRoads[RoadIdx]]
                for Channel in RoadObj.Cars:
                    if len(Channel):
                        if Channel[0].Position != 0:
                            Channel.insert(0, RoadGarage.pop(0))
                            Car = Channel[0]
                            Car.Position = min(Car.MaxVelocity, RoadObj.MaxVelocity,
                                               Channel[1].Position) - 1
                            Car.IfWait = False
                            Car.NextRoad = Car.Route[1]
                            CarHasAdd = True
                            break
                        else:
                            continue
                    else:
                        Channel.insert(0, RoadGarage.pop(0))
                        Car = Channel[0]
                        Car.Position = min(Car.MaxVelocity, RoadObj.MaxVelocity) - 1
                        Car.IfWait = False
                        Car.NextRoad = Car.Route[1]
                        CarHasAdd = True
                        break
                if CarHasAdd == False:
                    break

    def ScheduleRoads(self, RoadsList, CarObjHasEnd, LockCheckSymbol):
        self.WaitSchedule = False
        self.UpdateExitRoadsWaitSchedule(RoadsList)

        #一圈一圈地调度出路口，调完为止

        while(len(self.ExitRoadsWaitSchedule)):
            # 更新一个入口最大优先级表
            # print('sb', self.ExitRoadsWaitSchedule, self.ExitRoadsWaitScheduleMask)
            # 针对每条出口道路调度一圈
            for IdxInWaitSchedule, ExitRoad in enumerate(self.ExitRoadsWaitSchedule):
                self.UpdateEntrancePriority(RoadsList)
                ExitRoadIdx = self.ExitRoads.index(ExitRoad)
                ExitRoadObj = RoadsList[ExitRoad]
                # 将该条出口道路调度至优先级不够或对应入口路无法进车或已经调度完
                # print('sb')
                while(True):
                    # print('sb')
                    ExitChannel = ExitRoadObj.Cars[ExitRoadObj.WaitFirstPriority]
                    Car = ExitChannel[-1]
                    EntranceRoad = Car.NextRoad
                    if EntranceRoad == -1:
                        CarObjHasEnd.append(ExitChannel.pop(-1))
                        LockCheckSymbol = False
                        ExitRoadObj.UpdateTerminalStateChannel(ExitRoadObj.WaitFirstPriority, ExitChannel)
                        ExitRoadObj.UpdateFirstPriority()
                        if ExitRoadObj.WaitFirstPriority < 0:
                            self.ExitRoadsWaitScheduleMask[IdxInWaitSchedule] = False
                            break
                        continue
                        
                    EntranceRoadIdx = self.EntranceRoads.index(EntranceRoad)
                    EntranceRoadObj = RoadsList[EntranceRoad]
                    
                    # 如果当前转向的优先级等于入口最大优先级，则进行调度
                    # print('ddd', self.RoadsToTurnIdx[ExitRoadIdx, EntranceRoadIdx], self.EntrancePriority[EntranceRoadIdx])
                    # print('dddd', )
                    if self.RoadsToTurnIdx[ExitRoadIdx, EntranceRoadIdx] <= self.EntrancePriority[EntranceRoadIdx]:

                        MaxExitDistance = ExitRoadObj.Length - Car.Position - 1
                        MaxEntranceDistance = max(EntranceRoadObj.MaxVelocity - MaxExitDistance, 0)
                        # 速度限制使车辆无法调度到下条路的情况
                        if MaxEntranceDistance == 0:
                            Car.Position = ExitRoadObj.Length - 1
                            Car.IfWait = False
                            ExitRoadObj.UpdateTerminalStateChannel(ExitRoadObj.WaitFirstPriority, ExitChannel)
                            ExitRoadObj.UpdateFirstPriority()
                            if ExitRoadObj.WaitFirstPriority < 0:
                                self.ExitRoadsWaitScheduleMask[IdxInWaitSchedule] = False
                                break
                            continue
                        # 尝试向入口路的每个车道调车，按车道idx从小到大
                        for EntranceChannelIdx, EntranceChannel in enumerate(EntranceRoadObj.Cars):
                            if len(EntranceChannel) == 0 or EntranceChannel[0].Position >= MaxEntranceDistance:
                                EntranceChannel.insert(0, ExitChannel.pop(-1))
                                LockCheckSymbol = False
                                EntranceChannel[0].IfWait = False
                                EntranceChannel[0].UpdateStateToNextRoad(MaxEntranceDistance - 1)
                                ExitRoadObj.UpdateTerminalStateChannel(ExitRoadObj.WaitFirstPriority, ExitChannel)
                                ExitRoadObj.UpdateFirstPriority()
                                break
                            elif EntranceChannel[0].IfWait:
                                # 让出口路径变为不可调度
                                self.ExitRoadsWaitScheduleMask[IdxInWaitSchedule] = False
                                self.WaitSchedule = True
                                break
                            elif EntranceChannel[0].Position > 0:
                                EntranceChannel.insert(0, ExitChannel.pop(-1))
                                LockCheckSymbol = False
                                EntranceChannel[0].IfWait = False
                                EntranceChannel[0].UpdateStateToNextRoad(EntranceChannel[1].Position - 1)
                                ExitRoadObj.UpdateTerminalStateChannel(ExitRoadObj.WaitFirstPriority, ExitChannel)
                                ExitRoadObj.UpdateFirstPriority()
                                break
                            elif EntranceChannelIdx == EntranceRoadObj.NumChannel - 1:
                                # print('sb')
                                Car.Position = ExitRoadObj.Length - 1
                                Car.IfWait = False
                                ExitRoadObj.UpdateTerminalStateChannel(ExitRoadObj.WaitFirstPriority, ExitChannel)
                                ExitRoadObj.UpdateFirstPriority()
                                
                        # print(type(self.ExitRoadsWaitScheduleMask))
                        if ExitRoadObj.WaitFirstPriority < 0:
                            self.ExitRoadsWaitScheduleMask[IdxInWaitSchedule] = False
                            break
                        elif self.ExitRoadsWaitScheduleMask[IdxInWaitSchedule] == False:
                            break
                            
                    # 否则暂时放弃调度，调度下一个出口路
                    else:
                        # print('sb')
                        break
            self.ExitRoadsWaitSchedule = [self.ExitRoadsWaitSchedule[i] for i, Mask in enumerate(self.ExitRoadsWaitScheduleMask) if Mask]
            # self.ExitRoadsWaitSchedule = self.ExitRoadsWaitSchedule and self.ExitRoadsWaitScheduleMask
            self.ExitRoadsWaitScheduleMask = list(filter(lambda x:x, self.ExitRoadsWaitScheduleMask))
        # print(LockCheckSymbol, self.WaitSchedule)
        # print()
        return LockCheckSymbol and self.WaitSchedule


def FromFileToObj(RoadFile, CrossFile, CarFile, AnswerFile):

    with open(CarFile) as f:
        CarList = f.readlines()[1:]
        CarList = [eval(Car.strip('\n')) for Car in CarList]

    with open(RoadFile) as f:
        RoadList = f.readlines()[1:]
        RoadList = [eval(Road.strip('\n')) for Road in RoadList]

    with open(CrossFile) as f:
        CrossList = f.readlines()[1:]
        CrossList = [eval(Cross.strip('\n')) for Cross in CrossList]

    with open(AnswerFile) as f:
        AnswerList = f.readlines()[1:]
        AnswerList = [eval(Answer.strip('\n')) for Answer in AnswerList]

    CrossUniList = [Cross[0] for Cross in CrossList]
    CrossToIdx = dict([(Cross[0], CrossIdx) for CrossIdx, Cross in enumerate(CrossList)])
    CrossAdjacency = - np.ones((len(CrossList), len(CrossList)), np.int)
    CrossRoadToNext = []
    
    RoadToIdx = []
    RoadUniList = []
    RoadObjList = []
    # RoadObjList = [RoadClass(Road) for Road in RoadList]
    RoadIdx = 0
    for Road in RoadList:
        CrossRoadToNext.append(((Road[4], Road[0]), Road[5]))
        if Road[-1]:
            RoadUniList.append('p' + str(Road[0]))
            RoadToIdx.append(('p' + str(Road[0]), RoadIdx))
            RoadObjList.append(RoadClass(Road))
            CrossRoadToNext.append(((Road[4], Road[0]), Road[5]))
            CrossAdjacency[CrossToIdx[Road[4]], CrossToIdx[Road[5]]] = RoadIdx
            RoadIdx += 1
            RoadUniList.append('n' + str(Road[0]))
            RoadToIdx.append(('n' + str(Road[0]), RoadIdx))
            RoadObjList.append(RoadClass(Road))
            CrossRoadToNext.append(((Road[5], Road[0]), Road[4]))
            CrossAdjacency[CrossToIdx[Road[5]], CrossToIdx[Road[4]]] = RoadIdx
            RoadIdx += 1      
        else:
            RoadUniList.append('p' + str(Road[0]))
            RoadToIdx.append(('p' + str(Road[0]), RoadIdx))
            RoadObjList.append(RoadClass(Road))
            CrossRoadToNext.append(((Road[4], Road[0]), Road[5]))
            CrossAdjacency[CrossToIdx[Road[4]], CrossToIdx[Road[5]]] = RoadIdx
            RoadIdx += 1
    RoadToIdx = dict(RoadToIdx) 
    CrossRoadToNext = dict(CrossRoadToNext)
    CrossObjList = [CrossClass(RoadUniList, Cross, CrossToIdx, CrossAdjacency) for Cross in CrossList]

    CarObjList = [CarClass(Car, TimeAndRoute, CrossToIdx, CrossRoadToNext, CrossAdjacency) \
                  for Car, TimeAndRoute in zip(CarList, AnswerList)]

    CarObjToStart = CarObjList
    SortedStartCarIdx = np.array([Car.StartTime for Car in CarObjToStart]).argsort().tolist()
    CarObjToStart = [CarObjToStart[CarIdx] for CarIdx in SortedStartCarIdx]
    CarObjHasEnd = []
    
    return RoadObjList, CrossObjList, CarObjToStart, CarObjHasEnd

def AddCarToGarage(NowTime, CrossObjList, CarObjToStart):
    if len(CarObjToStart):
        Car = CarObjToStart[0]
        while (NowTime == Car.StartTime):
            CrossObjAdd = CrossObjList[Car.StartCross]
            StartRoad = Car.Route[0]
            StartRoadIdx = CrossObjAdd.EntranceRoads.index(StartRoad)
            CrossObjAdd.UnlimitedGarage[StartRoadIdx].append(CarObjToStart.pop(0))
            if len(CarObjToStart):
                Car = CarObjToStart[0]
            else:
                break


if __name__ == '__main__':

    FileDir = 'config_5'

    RoadFile = FileDir + '/road.txt'
    CrossFile = FileDir + '/cross.txt'
    CarFile = FileDir + '/car.txt'
    AnswerFile = FileDir + '/answer.txt'

    RoadObjList, CrossObjList, CarObjToStart, CarObjHasEnd = FromFileToObj(RoadFile, CrossFile, CarFile, AnswerFile)
    CarNum = len(CarObjToStart)
    NowTime = 0
    '''
    for CrossObj in CrossObjList:
        print(CrossObj.EntranceRoads)
    '''
    t0 = time.time()
    while(len(CarObjHasEnd) < CarNum):
        # print('NowTime', NowTime)

        for CrossObj in CrossObjList:
            CrossObj.WaitSchedule = True

        for RoadObj in RoadObjList:
            RoadObj.SetAllCarWait()
            RoadObj.UpdateTerminalStateRoad()

        NeedScheduleCrossNum = len(CrossObjList)
        while(NeedScheduleCrossNum):
            # print(NeedScheduleCrossNum)
            LockCheckSymbol = True
            NeedScheduleCrossNum = 0
            # print('sb')
            for CrossObj in CrossObjList:
                if CrossObj.WaitSchedule == True:
                    LockCheckSymbol = CrossObj.ScheduleRoads(RoadObjList, CarObjHasEnd, LockCheckSymbol) and LockCheckSymbol
                    NeedScheduleCrossNum += int(CrossObj.WaitSchedule)

                # print(LockCheckSymbol)
            if LockCheckSymbol:
                DeadLockCross = []
                for CrossIdx, CrossObj in enumerate(CrossObjList):
                    if CrossObj.WaitSchedule == True:
                        DeadLockCross.append(CrossObj.CrossNum)
                # print(LockCheckSymbol)
                raise Exception('ErrorDeadLock, The locked crosses are: ' + str(DeadLockCross))

        AddCarToGarage(NowTime, CrossObjList, CarObjToStart)
        for CrossObj in CrossObjList:
            CrossObj.AddCarFromGarage(RoadObjList)
        # print(len(CarObjHasEnd))
        NowTime += 1
        print(NowTime, len(CarObjHasEnd))
    t1 = time.time()
    print(NowTime)
    print(t1 - t0)

    
