%% Performs a line-road simluation of some number of vehicles:

number_vehicles = 100;
simulation_length = 75;
dt = .1;
numSteps = round(simulation_length/dt);

init_spacing = 20;

lead_speed = V(init_spacing); % Need to have optimal velocity func in work space

leader_speeds = ones(numSteps)*lead_speed;

follower_speeds = zeros(number_vehicles,numSteps);
positions = zeros(number_vehicles,numSteps);
spacings = zeros(number_vehicles,numSteps);


%% Individual vehicle driving parameters:
b = 20; 
a = 0.5; 
vm = 20; 
d0 = 4.5;
x = 1.5;
y = 2;

wantUniform = true; % Gives a homogeneous platoon

Params = zeros(number_vehicles,6);
if(wantUniform)
    for i=1:number_vehicles
        Params(i,:) = [b,a,vm,d0,x,y];
    end
else 
    % Adds noise around the parameter set to create variability in driving
    % behavior:
    for i=1:number_vehicles
        b_i = b + normrnd(0,1);
        a_i = a + normrnd(0,.1);
        vm_i = vm + normrnd(0,1.5);
        d0_i = d0 + normrnd(0,.3);
        x_i = x + normrnd(0,.05);
        y_i = y + normrnd(0,.05);
        Params(i,:) = [b_i,a_i,vm_i,d0_i,x_i,y_i];
    end
end


%% Perturb the leader speed:


t_decel = 5;
amount_decel = (lead_speed-1)/t_decel;

if(time >= 0 && time <= t_decel)
    accel_vals(1) = -amount_decel;
    disp('Causing Breaking Event')
end


%% Perform the simulation:

time = 0;
for t=1:numSteps
    %% Do integration steps:
    clc
    disp(t)
    time = time + dt;
    accel_Vals = zeros(number_vehicles,1);
    
    accel_Vals
   
    
    stateMat(:,2) = stateMat(:,2)+accel_vals*dt;
    stateMat(:,1) = stateMat(:,1)+stateMat(:,2)*dt;
     
    for carNum=1:n
        if(stateMat(carNum,1)>L)
         stateMat(carNum,1) = stateMat(carNum,1)-L;
        end
    end
    
    posVals(:,t) = stateMat(:,1);
    speedVals(:,t) = stateMat(:,2);
    
    

end



