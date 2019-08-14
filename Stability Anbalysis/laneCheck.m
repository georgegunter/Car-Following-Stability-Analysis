function [laneMat] = laneCheck(Params,laneMat,stateMat,RingRoad_Params,changing_params)
%Determines whether cars in either lane want to switch to the other lane
%and then returns a new lane Matrix that has updated which lane they are in
%depending on their decision:

%% Safety Check:

n = RingRoad_Params.number_cars;%number of cars
dt = RingRoad_Params.dt;

%Calculate distances behind and in front of each car in the opposite lane: 

distanceBehindOtherLane = zeros(n,1);

for carNum=1:n
   %Find the distance behind where would be in that lane:
   [d,~] = getDistBehind(stateMat,laneMatcarNum);
   distanceBehindOtherLane(carNum) = d;
end


%% Incentive Check:

accelCurr = zeros(n,1);
accelOtherLane = zeros(n,1);

for carNum=1:n
    laneNum = 1;
    otherLane = 2;
    if(~laneMat(carNum,laneNum))
       laneNum = 2;
       otherLane = 1;
    end

    p = Params(carNum,:);

    accelCurr(carNum) = accelCalc(p,laneMat,stateMat,laneNum,carNum);
    accelOtherLane(carNum) = accelCalc(p,laneMat,stateMat,otherLane,carNum);
end



gamma = changing_params(1); %Incentive factor to determine whether or not to switch lanes

safetyDist = changing_params(2); %Amount need to have in front of car behind in other lane

velOrig = stateMat(:,2);

currVel = velOrig + accelCurr*dt;

newVel = velOrig + accelOtherLane*dt;%Do a velocity increment


%% Do Lane Update

for carNum=1:n
    if((newVel(carNum) > currVel(carNum)*gamma) &&...
            distanceBehindOtherLane(carNum) >= safetyDist)
        
      laneMat(carNum,:) = ~laneMat(carNum,:);
    end
end



end

