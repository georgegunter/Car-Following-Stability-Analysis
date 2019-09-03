
clc
close all
%% Contains the LineRoad parameters:
n = 200;
simulation_time = 250;
dt = .1;
numSteps = round(simulation_time/dt);


LineRoad_Params.number_cars = n;
LineRoad_Params.simulation_time = simulation_time;
LineRoad_Params.dt = dt;
LineRoad_Params.numSteps = numSteps;

%% Define Simulation Variables:

b=20;a=.5;vm=9.75;d0=2.23;
params = [b,a,vm,d0];
V = @(d) vm*(tanh(d./d0-2)+tanh(2))/(1+tanh(2));

s_init = 10*ones(n,1);
v_init = V(s_init);
p_init = -cumsum(s_init);
p_init = p_init - p_init(end);
 

v_lead = ones(1,numSteps).*v_init;
v_lead(1:length(dip_vals)) = v_lead(1:length(dip_vals)) + dip_vals';


%% Run Simultation
[Speeds,Positions,Spacings] = ...
    Run_Line_Road(LineRoad_Params,params,s_init,p_init,v_init,v_lead);


%% Plot Results:
figure()
plot(Spacings')
title('Spacings')

figure()
plot(Speeds')
title('Speeds')


