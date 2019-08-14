function [accel_vals ] = accelCalc(Params,laneMat,stateMat,RingRoad_Params)
%Calculates a new acceleration for a specific car. Uses the optimal
%velocity function and acceleration function:


[~,s_vals] = getDistBehindAndFront(stateMat,laneMat,RingRoad_Params);

accel_vals = zeros(RingRoad_Params.number_cars,1);

for c=1:RingRoad_Params.number_cars
    v = stateMat(c,2);
    v_l = 0;
    if(c==RingRoad_Params.number_cars)
        v_l = stateMat(1,2);
    else
        v_l = stateMat(c+1,2);
    end 
    s = s_vals(c);
    p = Params(c,:);
    accel_vals(c) = Bando_FTL_Accel(p,v,v_l,s);
end

end
