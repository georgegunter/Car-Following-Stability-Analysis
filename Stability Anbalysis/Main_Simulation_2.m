clear;clc

%% Contains the ringroad parameters:
RingRoad_Params.Road_Length = 75;
RingRoad_Params.number_cars = 10;
RingRoad_Params.number_lanes = 2;
RingRoad_Params.trajectory_time = 75;
RingRoad_Params.dt = .1;


L = RingRoad_Params.Road_Length;
n = RingRoad_Params.number_cars;
number_lanes = RingRoad_Params.number_lanes;
tf = RingRoad_Params.trajectory_time; % final time of trajectory computation
dt = RingRoad_Params.dt; % Time step [s]
numSteps = tf/dt;

%% Individual vehicle driving parameters:
b = 20; 
a = 0.5; 
vm = 9.72; 
d0 = 4.5;
x = 1.5;
y = 2;

wantUniform = true; % Gives a homogeneous platoon

Params = zeros(n,6);
if(wantUniform)
    for i=1:n
        Params(i,:) = [b,a,vm,d0,x,y];
    end
else 
    % Adds noise around the parameter set to create variability in driving
    % behavior:
    for i=1:n
        b_i = b + normrnd(0,1);
        a_i = a + normrnd(0,.1);
        vm_i = vm + normrnd(0,1.5);
        d0_i = d0 + normrnd(0,.3);
        x_i = x + normrnd(0,.05);
        y_i = y + normrnd(0,.05);
        Params(i,:) = [b_i,a_i,vm_i,d0_i,x_i,y_i];
    end
end

%% Initial Conditions:


s_star = RingRoad_Params.Road_Length/RingRoad_Params.number_cars;

v_star = vm*(tanh(s_star./d0-2)+tanh(2))/(1+tanh(2));

initial_spacings = ones(n,1)*s_star;

stateMat = zeros(n,2); % Form: [position,velocity]
stateMat(:,1) = cumsum(initial_spacings);
stateMat(:,2) = v_star;

laneMat = ones(n,1);

posVals = zeros(n,numSteps);
speedVals = zeros(n,numSteps);

want_Lane_Chage = false;

lane_check_frequency = 5; %How frequently a vehicle will check to see if it wants to change lanes

accel_vals = zeros(n,1);

%% Plotting:

circumfReal = L;
radReal = circumfReal/(2*pi);
delta = 5;
circumferenceOutter = circumfReal;
radOutter = circumferenceOutter/(2*pi)+delta;
circumferenceInner = circumfReal;
radInner = circumferenceInner/(2*pi)-delta;

posMatrix = zeros(n,2);

inner = zeros(100,2);
middle = zeros(100,2);
outter = zeros(100,2);

for i=1:100
    theta = 2*pi*(i/100);
    inner(i,1) = cos(theta)*(radInner-delta);
    inner(i,2) = sin(theta)*(radInner-delta);
    outter(i,1) = cos(theta)*(radOutter+delta);
    outter(i,2) = sin(theta)*(radOutter+delta);
    middle(i,1) = cos(theta)*(radReal);
    middle(i,2) = sin(theta)*(radReal);
end

colorGradient = linspace(1,10,n);

close all;
figure
hold on;
plot(inner(:,1),inner(:,2),'k-')
plot(outter(:,1),outter(:,2),'k-')
plot(middle(:,1),middle(:,2),'b--')
xlim([-radOutter-10,radOutter+10])
ylim([-radOutter-10,radOutter+10])

%% Simulation:

time = 0;
for t=1:numSteps
    %% Do integration steps:
    clc
    disp(t)
    time = time + dt;
    
    if(want_Lane_Chage)
   
        if(mod(time,lane_check_frequency)==0)
            laneMat = laneCheck(Params,laneMat,stateMat,changing_params,dt);
        end
        
    end
    
    accel_vals = accelCalc(Params,laneMat,stateMat,RingRoad_Params);
    
    t_decel = 2;
    
    if(t >= 1 && t <= t_decel)
        accel_vals(1) = -(v_star/t_decel);
        disp('Causing Breaking Event')
    end
    
    stateMat(:,2) = stateMat(:,2)+accel_vals*dt;
    stateMat(:,1) = stateMat(:,1)+stateMat(:,2)*dt;
     
    for carNum=1:n
        if(stateMat(carNum,1)>L)
         stateMat(carNum,1) = stateMat(carNum,1)-L;
        end
    end
    
    posVals(:,t) = stateMat(:,1);
    speedVals(:,t) = stateMat(:,2);
    
    
    %% Put everything into the circular track: 
    for carNum=1:n
        theta = (stateMat(carNum,1)/circumfReal)*2*pi;
        if(laneMat(carNum,1))
            posMatrix(carNum,1) = cos(theta)*radOutter;
            posMatrix(carNum,2) = sin(theta)*radOutter;
        else
            posMatrix(carNum,1) = cos(theta)*radInner;
            posMatrix(carNum,2) = sin(theta)*radInner;
        end
    end
end

%% Display the ring-road simulation:
time = 0;
for t=1:numSteps
    time = time + RingRoad_Params.dt;
    cla;
    %plot(posMatrix(:,1),posMatrix(:,2),'ro');%This actually plots the cars
    scatter(posMatrix(:,1),posMatrix(:,2),50,colorGradient,'filled');
    %Draw lines to denote lanes:
    plot(inner(:,1),inner(:,2),'k-');
    plot(outter(:,1),outter(:,2),'k-');
    plot(middle(:,1),middle(:,2),'b--');
    v_bar = mean(stateMat(:,2));
    title(strcat('Mean Speed:',num2str(v_bar)))
    
    pause(.05)
end

%% Plot Phase Diagram:

figure()
plot(posVals','b.','MarkerSize',5)
ylim([0 RingRoad_Params.Road_Length])

%% String Stability Diagram:
speed_diff_vals = zeros(n,numSteps);
spacing_vals = zeros(n,numSteps);

for c=1:n
    if(c==n-1)
        speed_diff_vals(c,:) = speedVals(n,:) - speedVals(c,:);
        spacing_vals(c,:) = posVals(n,:) - posVals(c,:);
        spacing_vals(c,:) = mod(spacing_vals(c,:),L);
    else
        speed_diff_vals(c,:) = speedVals(mod(c+1,n),:) - speedVals(c,:);
        spacing_vals(c,:) = posVals(mod(c+1,n),:) - posVals(c,:);
        spacing_vals(c,:) = mod(spacing_vals(c,:),L);
    end
end

figure()
subplot(3,1,1)
hold on

for c=1:n
    p= Params(c,:);
    lambda_vals = string_stability(p,10,10,spacing_vals(c,:));
    if(c==1)
        plot(lambda_vals,'r.')
    else
        plot(lambda_vals,'b.')
    end
end

subplot(3,1,2)
hold on

for c=1:n
    if(c==1)
        plot(spacing_vals(c,:),'r.')
    else
        plot(spacing_vals(c,:),'b.')
    end
end

min_s = min(spacing_vals,[],'all');
max_s = max(spacing_vals,[],'all');
s_range = min_s:.05:max_s;

lambda_range = string_stability(p,10,10,s_range);

subplot(3,1,3)
hold on
plot(s_range,lambda_range)
plot(s_star,string_stability(p,10,10,s_star),'k*','MarkerSize',10)





