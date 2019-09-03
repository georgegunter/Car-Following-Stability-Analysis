function [Speeds,Positions,Spacings] = Run_Line_Road(LineRoad_Params,params,s_init,p_init,v_init,v_lead)
%% Specify Ring Road:
n = LineRoad_Params.number_cars;
tf = LineRoad_Params.simulation_time; % final time of trajectory computation
dt = LineRoad_Params.dt; % Time step [s]
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
    v_l(1) = v_lead(t);
    v_l(2:end) = v(1:end-1);
   
    a = Bando_FTL_Accel(params,v,v_l,s); 
    
    p = p+v*dt;
    
    v = v+a*dt;% Euler Step
    
    s = s+(v_l-v)*dt;
    
    % Add Noise to purturb state from equilibrium:
    if(mod(t,round(1/dt))==0 && (t < round(numSteps/2)))
        p = p + randntrunc(1,n,3)'*.1;
    end

%     if(mod(t,round(1/dt))==0 && (time < 10))
%         p = p + randntrunc(1,n,3)'*.5;
%     end
    
    
    Positions(:,t) = p;
    Speeds(:,t) = v;
    Spacings(:,t) = s;   
        
end

end

