function [distance_behind] = getDistBehind(stateMat,laneMat,RingRoad_Params)
%Finds the distance to the car in front of a given car index:

n = RingRoad_Params.number_cars;%number of cars
TrackLength = RingRoad_Params.Road_Length; % length of circular road in meters
maxDist = TrackLength;%Maximum possible distance -> gets updated
distance_behind = zeros(n,1);

for i=1:n
    currPos = stateMat(i,1);
    currLane = laneMat(i);
    d = maxDist;
    for j=1:n
        if(i ~= j)
            if(laneMat(j) == currLane)
                
                
            end
        end
    end

end
            


end