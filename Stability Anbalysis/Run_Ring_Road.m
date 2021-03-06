function [Speeds,Positions,Spacings] = Run_Ring_Road(RingRoad_Params,params,s_init,p_init,v_init)
%% Specify Ring Road:
L = RingRoad_Params.Road_Length;
n = RingRoad_Params.number_cars;
tf = RingRoad_Params.trajectory_time; % final time of trajectory computation
dt = RingRoad_Params.dt; % Time step [s]
numSteps = tf/dt;


%% Initialize:

p = p_init;
v = v_init;
s = s_init;

Positions = zeros(n,numSteps);
Speeds = zeros(n,numSteps);
Spacings = zeros(n,numSteps);

Positions(:,1) = p;
Speeds(:,1) = v;
Spacings(:,1) = s;

%% Perform Simulation:

time = 0;
for t=2:numSteps
    %% Do integration steps:
    time = time + dt;
    
%     t_decel = 10;
%     amount_decel = (v_star-.1)/t_decel;
%     if(time >= 0 && time <= t_decel)
%         accel_vals(1) = -amount_decel;
%         disp('Causing Breaking Event')
%     end     
    
    % Last index follows first
    v_l = zeros(n,1);
    v_l(end) = v(1);
    v_l(1:end-1) = v(2:end);
   
    a = Bando_FTL_Accel(params,v,v_l,s); 
    
    p = p+v*dt;
    
    v = v+a*dt;% Euler Step
    
    % Add Noise to purturb state from equilibrium:
%     if(mod(t,round(1/dt))==0 && (t < round(numSteps/2)))
%         p = p + randntrunc(1,n,3)'*.1;
%     end

    if(mod(t,round(1/dt))==0 && (time < 10))
        p = p + randntrunc(1,n,3)'*.5;
    end

    p = mod(p,L);
    
    for c=1:n-1
        s(c) = mod(p(c+1) - p(c),L);
    end
    
    s(end) = mod(p(1) - p(end),L);
    
    
    Positions(:,t) = p;
    Speeds(:,t) = v;
    Spacings(:,t) = s;   
        
end

end

