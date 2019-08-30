%% Load in ARED Data:
clc
% cd('ARED Data - Experiment A\')
ARED_Data = cell(20,1);
for v=1:20
    file_name = strcat('Vehicle_',num2str(v),'_Experiment_A.csv');
    data = readmatrix(file_name);
    ARED_Data{v} = data;
    disp(length(data));
end
% cd('..')
%% Arrange into matrices:

% Speeds_ARED = 