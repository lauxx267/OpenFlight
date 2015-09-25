% Simulation Init
%
% Authors:
% Taylor, Brian R (BRT)
% brtaylor@umn.edu
%
% Revision History
% 2015-09-25  Basline     BRT
%

function []=init(varargin)

%% adding paths
addpath(genpath('flightTestVehicle'));
addpath(genpath('groundControlStation'));
addpath(genpath('pilotInterface'));
addpath(genpath('libraries'));

%% kicking off child init files

