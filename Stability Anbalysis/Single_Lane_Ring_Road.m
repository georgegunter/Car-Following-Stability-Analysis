%% Contains the ringroad parameters:

Road_Length = 230;
number_cars = 22;
simulation_time = 75;
dt = .1;
numSteps = round(simulation_time/dt);


RingRoad_Params.Road_Length = Road_Length;
RingRoad_Params.number_cars = number_cars;
RingRoad_Params.simulation_time = simulation_time;
RingRoad_Params.dt = dt;
RingRoad_Params.numSteps = numSteps;

%% Define Simulation Variables:

b=20;a=.5;vm=9.72;d0=2.23;
Bando_Params = [b,a,vm,d0];
V = @(d) vm*(tanh(d./d0-2)+tanh(2))/(1+tanh(2));


Speed_Values = zeros(number_cars,numSteps);
Position_Values = zeros(number_cars,numSteps);
Spacing_Values = zeros(number_cars,numSteps);

Spacing_Values(:,1) = Road_Length/number_cars;
Speed_Values(:,1) = V(Road_Length/number_cars);
Position_Values(:,1) = cumsum(Spacing_Values(:,1));

wantNoise = true;


%% Run through the simulation:

% Car 1 follows car n:

for t=2:numSteps
    
    
    v = Speed_Values(:,t-1);
    v_l = v*0;
    v_l(1) = v(end);
    v_l(2:end) = v(1:end-1);  
    s = Spacing_Values(:,t-1);
    p = Position_Values(:,t-1);
    
    [dV,dS,dP] = Bando_FTL_RK_Step(RingRoad_Params,Bando_Params,v,v_l,s,p);
    
    v_new = v + dV;
    s_new = s + dS;
    p_new = p + dP;
    
    
    if(wantNoise):
        
    
    Speed_Values(:,t) = v_new;
    Spacing_Values(:,t) = s_new;
    Position_Values(:,t) = p_new;
    
end

%% Plot Results:

figure()
plot(
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
end

%% Plot Results:


