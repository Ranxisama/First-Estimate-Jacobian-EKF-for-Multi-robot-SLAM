close all

clear

Config;

currentFolder = fileparts(mfilename('fullpath'));
ekfslamv2Folder = fullfile(currentFolder,'ekfslam_v2');
addpath(ekfslamv2Folder)

if env == 1
    R1XState = fullfile(ekfslamv2Folder,'R1_XState_1.mat');
    R2XState = fullfile(ekfslamv2Folder,'R2_XState_1.mat');
elseif env == 2
    R1XState = fullfile(ekfslamv2Folder,'R1_XState_2.mat');
    R2XState = fullfile(ekfslamv2Folder,'R2_XState_2.mat');
end

if exist(R1XState,'file')
    load(R1XState);
    lm1 = lm;
    wp1 = wp;
    fprintf('R1_XState loaded, saved as: %s\n', R1XState)
else
    error('R1_XState not found');
end
if exist(R2XState,'file')
    load(R2XState);
    lm2 = lm;
    wp2 = wp;
    fprintf('R2_XState loaded, saved as: %s\n', R2XState)
else
    error('R2_XState not found');
end

wp = [wp1,wp2];
lm = [lm1,lm2];

TrueStatePath = fullfile(currentFolder,'TrueState');
if ~exist(TrueStatePath,'dir')
    mkdir(TrueStatePath);
end
UncheckedTrueStatePath = fullfile(TrueStatePath,'UncheckedTrueState.mat');
RobotsWaypointsPath = fullfile(TrueStatePath,'RobotsWaypoints.mat');

save(UncheckedTrueStatePath,'wp','lm')
save(RobotsWaypointsPath,'wp1','wp2')

% run 'frontend' in command window and load 'currentFolder\TState\UncheckedTrueState.m' to: 
% 1. check waypoints of R1 and R2, and features
% 2. edit waypoints
% 2. remove features that overlay with waypoints
% 4. save the edited X state as 'currentFolder\TrueState\CheckedTrueState.mat'
frontend;
fprintf('UncheckedTrueState and RobotsWaypoints saved to: %s\n', TrueStatePath)