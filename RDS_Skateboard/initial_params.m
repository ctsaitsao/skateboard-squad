%% initial_params.m
%
% Description:
%   Initializes the values of many parameters, such as parameters in the
%   system dynamics, parameters that relate to simulating the system
%   forward in time, and parametes that relate to visualization/animation.
%   
% Inputs:
%   none

% Outputs:
%   params: a struct with many elements


function params = initial_params

% Skateboard Parameters

    params.boardMass = 0.85; % kg
    params.boardAngleInit = 0; % radians
    params.boardInert = 0;
    params.boardXInit = 0;
    
    params.boardLength = 0.4;  % m
    params.boardHeight = 0.063;
    
% Bottom Link Parameters

    params.bottomLinkMass = 1.124;
    params.bottomLinkInert = 0;
    params.bottomLinkTheta = 0; % wrt the skateboard, with 0 being positive x
    params.bottomLinkWidth = 0.04;
    params.bottomLinkHeight = .2;
    params.bottomLinkXCOM = 0;
    params.bottomLinkYCOM = .107; % m, taken wrt the joint with the skateboard
    
    
% Top Link Parameters

    params.topLinkMass = 1.498; % kg
    params.topLinkInert = 0;
    params.topLinkTheta = 0; % wrt the first link, with 0 being stright up (positive y)
    params.topLinkWidth = 0.085; % m, average of wide and narrow parts of top link
    params.topLinkHeight = .3;
    params.topLinkXCOM = 0;
    params.topLinkYCOM = 0.137;
    
    params.viz.colors.board = [1 0 0];
    params.viz.colors.bottomLink = [0 1 0];
    params.viz.colors.topLink = [0 0 1];
    params.viz.colors.bottomLinkCoM = [0.75 0.75 0.75];
    params.viz.colors.tracers.boardCoM = 'r';
    params.viz.colors.tracers.bottomLinkCoM = 'g';
    params.viz.colors.tracers.topLinkCom = 'b';
    params.viz.axis_lims = [-3,3,-2,2];
  
end