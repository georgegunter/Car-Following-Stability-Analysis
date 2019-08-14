function [distance_behind_vals,distance_infront_vals] = getDistBehindAndFront(stateMat,laneMat,RingRoad_Params)
%

n = RingRoad_Params.number_cars;%number of cars
TrackLength = RingRoad_Params.Road_Length; % length of circular road in meters
maxDist = TrackLength;%Maximum possible distance -> gets updated

distance_behind_vals = zeros(n,1);
distance_infront_vals = zeros(n,1);

for i=1:n
    currPos = stateMat(i,1);
    currLane = laneMat(i);
    distance_behind = maxDist;
    distance_infront = maxDist;
    for j=1:n
        if(i ~= j)
            if(laneMat(j) == currLane)
                pos_other_vehicle = stateMat(j,1);
                curr_dist_infront = pos_other_vehicle-currPos;
                if(curr_dist_infront < 0)
                    curr_dist_infront = TrackLength+curr_dist_infront;
                end
                curr_dist_behind = TrackLength-curr_dist_infront;
                
                if(curr_dist_infront < distance_infront)
                    distance_infront = curr_dist_infront;
                end
                if(curr_dist_behind < distance_behind)
                    distance_behind = curr_dist_behind;
                end
            end
        end
    end
    distance_behind_vals(i) = distance_behind;
    distance_infront_vals(i) = distance_infront;
end
            


end