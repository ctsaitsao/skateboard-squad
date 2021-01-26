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
    params.boardXInit = 0;
    params.boardLength = 0.4;  % m
    params.boardHeight = 0.063;
    params.boardI = params.boardMass*(params.boardLength^2 + params.boardHeight^2)/12; %% Colgate replaced 
    params.wheelRadius = 0.03;
    
% Bottom Link Parameters

    params.bottomLinkMass = 1.124;
    params.bottomLinkTheta = 0; % wrt the skateboard, with 0 being positive x
    params.bottomLinkWidth = 0.04;
    params.bottomLinkHeight = .2;
    params.bottomLinkI = params.bottomLinkMass*(params.bottomLinkWidth^2 + params.bottomLinkHeight^2)/12; %% Colgate replaced
    params.bottomLinkXCoM = 0;
    params.bottomLinkYCoM = .107; % m, taken wrt the joint with the skateboard
    
    
% Top Link Parameters

    params.topLinkMass = 2; % kg
    params.topLinkTheta = 0; % wrt the first link, with 0 being stright up (positive y)
    params.topLinkWidth = 0.085; % m, average of wide and narrow parts of top link
    params.topLinkHeight = .3;
    params.topLinkI = params.topLinkMass*(params.topLinkWidth^2 + params.topLinkHeight^2)/12; % Colgate replaced
    params.topLinkXCoM = 0;
    params.topLinkYCoM = 0.2;
    
    params.sim.dt = 0.05;
    dt = params.sim.dt;
    params.sim.tfinal = 1;
    tfinal = params.sim.tfinal;
    
    params.g = 10;
    
    params.bottomMotor.maxTorque = 30.6; % (Nm)
    params.topMotor.maxTorque = 16.2; % (Nm)
    
    params.bottomMotor.maxDTheta = 121.7 * 0.10472; % rpm converted to radians per second
    params.topMotor.maxDTheta = 243.5 * 0.10472; % rpm converted to radians per second
    
    params.bottomMotor.time = 0:dt:tfinal;
    params.topMotor.time = 0:dt:tfinal;
    
    params.bottomMotor.torque = zeros(1, length(params.bottomMotor.time));
    params.topMotor.torque = zeros(1, length(params.topMotor.time));
     
    params.sim.constraints = logical([1, 1]); % left and right
    params.sim.restitution = [0.0, 0.0];
    params.sim.stage = 'ramp';
    params.sim.trick = 'idle';
    params.sim.gain = 500;
    
    params.trackRadius = 2;
    
    
    params.viz.colors.board = [1 0 0];
    params.viz.colors.bottomLink = [0 1 0];
    params.viz.colors.topLink = [0 0 1];
    params.viz.colors.boardCoM = [0 0.5 0.5];
    params.viz.colors.bottomLinkCoM = [0.75 0.75 0];
    params.viz.colors.topLinkCoM = [0.5 0 0.5];
    params.viz.colors.robotCoM = 'cyan';
    params.viz.colors.trackers = [0.2 0 0];

    params.viz.axis_lims = [-1,1,-1,1];
    
    
  
end