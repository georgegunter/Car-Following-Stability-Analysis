function [Speeds,Positions,Spacings] = Run_Ring_Road(RingRoad_Params,Params,p_init,v_init)
%% Specify Ring Road:
L = RingRoad_Params.Road_Length;
n = RingRoad_Params.number_cars;
tf = RingRoad_Params.trajectory_time; % final time of trajectory computation
dt = RingRoad_Params.dt; % Time step [s]
numSteps = tf/dt;


%% Initialize:

stateMat(:,1) = p_init;
stateMat(:,2) = v_init;

Positions = zeros(n,numSteps);
Speeds = zeros(n,numSteps);
Spacings = zeros(n,numSteps);

%% Perform Simulation:

time = 0;
for t=1:numSteps
    %% Do integration steps:
    time = time + dt;
    
    accel_vals = accelCalc(Params,laneMat,stateMat,RingRoad_Params);    
    
%     t_decel = 10;
%     amount_decel = (v_star-.1)/t_decel;
%     if(time >= 0 && time <= t_decel)
%         accel_vals(1) = -amount_decel;
%         disp('Causing Breaking Event')
%     end     
    


    stateMat(:,2) = stateMat(:,2)+accel_vals*dt;% Euler Step
    
    
    stateMat(:,1) = stateMat(:,1)+stateMat(:,2)*dt;
 
    if(mod(t,round(1/dt))==0 && (t < round(numSteps/2)))
        stateMat(:,1) = stateMat(:,1) + randntrunc(1,n,3)'*.1;
    end
    
     
    for carNum=1:n
        if(stateMat(carNum,1)>L)
         stateMat(carNum,1) = stateMat(carNum,1)-L;
        end
    end
    
    Positions(:,t) = stateMat(:,1);
    Speeds(:,t) = stateMat(:,2);
    for c=1:n-1
        Spacings(c,t) = mod(Positions(c+1,t) - Positions(c,t),L);
    end
    
    Spacings(end,t) = mod(Positions(c+1,t) - Positions(c,t),L);
            
        
        
end

end

