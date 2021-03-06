%% Performs a line-road simluation of some number of vehicles:
clc
number_vehicles = 22;
dt = .01;



omega = .01;

simulation_time = 2*pi/omega*5;


numSteps = round(simulation_time/dt);

b = 20; 
a = 0.5; 
vm = 9.72; 
d0 = 2.23;

V = @(d) vm*(tanh(d./d0-2)+tanh(2))/(1+tanh(2));

init_spacing = 1;

lead_speed = V(init_spacing); % Need to have optimal velocity func in work space
% 
% leader_speeds = ones(1,numSteps)*lead_speed;
% leader_speeds = leader_speeds + normrnd(0,.05,size(leader_speeds));


leader_speeds = sin(linspace(0,simulation_time,numSteps)*omega)+lead_speed;

follower_speeds = zeros(number_vehicles,numSteps);
positions = zeros(number_vehicles,numSteps);
spacings = zeros(number_vehicles,numSteps);

follower_speeds(:,1) = leader_speeds(1);
spacings(:,1) = init_spacing;
positions(:,1) = fliplr(cumsum(spacings(:,1))')';


%% Individual vehicle driving parameters:

wantUniform = true; % Gives a homogeneous platoon

Params = zeros(number_vehicles,4);
if(wantUniform)
    for i=1:number_vehicles
        Params(i,:) = [b,a,vm,d0];
    end
else 
    % Adds noise around the parameter set to create variability in driving
    % behavior:
    for i=1:number_vehicles
        b_i = b + normrnd(0,1);
        a_i = a + normrnd(0,.1);
        vm_i = vm + normrnd(0,1.5);
        d0_i = d0 + normrnd(0,.3);
        Params(i,:) = [b_i,a_i,vm_i,d0_i];
    end
end


%% Perform simulation:

time = 0;
for t=2:numSteps
    %% Do integration steps:
      
    
    v = follower_speeds(:,t-1);
    v_l = zeros(number_vehicles,1);
    v_l(1) = leader_speeds(t-1);
    v_l(2:end) = follower_speeds(1:end-1,t-1);
    s = spacings(:,t-1);
    p = positions(:,t-1);
    
%     [dV,dS,dP] = Bando_FTL_RK_Step(number_vehicles-1,dt,Params(1,:),v,v_l,s);
    dV = Bando_FTL_Accel(Params(1,:),v,v_l,s)*dt;
    dS = (v_l - v)*dt;
    dP = v*dt;

    v_new = v+dV;
    s_new = s+dS;
    p_new = p+dP;

    spacings(:,t) = s_new;
    follower_speeds(:,t) = v_new;
    positions(:,t) = p_new;

end


%% Plot Results:

figure()
hold on
plot(follower_speeds','b','MarkerSize',1);
plot(leader_speeds,'r','MarkerSize',2);


figure()
plot(spacings','b','MarkerSize',1);


