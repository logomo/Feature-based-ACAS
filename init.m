%% Initialization of the folder structure ##RUN FIRST##
% Note - Matlab does not support packages like java or other OO languages
clear all
s=pwd;
addpath(s)
addpath([s,'\VehicleModel']);
addpath([s,'\MissionControl']);
addpath([s,'\Tests'])
addpath([s,'\Enumerations']);
addpath([s,'\AvoidanceGrid']);
addpath([s,'\Utilities']);
addpath([s,'\AdversaryVehicle'])
addpath([s,'\Obstacles'])
addpath([s,'\Djikstra'])
addpath([s,'\RuleEngine'])
addpath([s,'\UavTraficManagement'])
addpath([s,'\External\C130'])
addpath([s,'\Scenarios'])
addpath([s,'\Presentation'])
clear