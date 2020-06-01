%% conservative_forces.m
%
% Description:
%   Wrapper function for autogen_conservative_forces.m
%   Computes the gravitational and elastic forces on the robot
%
% Inputs:
%   x: the state vector, x = [q; q_dot];
%   params: a struct with many elements, generated by calling init_params.m
%
% Outputs:
%   G = values of three conservative forces

function [G] = conservative_forces(x,params)

boardMass = params.boardMass;
boardTheta = x(1);
boardHeight = params.boardHeight/2;  % divide by 2 because jumping robot example measures height to center of mass
bottomLinkRCoM = params.bottomLinkYCoM;
bottomLinkMass = params.bottomLinkMass;
bottomLinkTheta = x(2);
bottomLinkHeight = params.bottomLinkHeight;
g = params.g;
topLinkRCoM = params.topLinkYCoM;
topLinkMass = params.topLinkMass;
topLinkTheta = x(3);

G = autogen_conservative_forces(boardMass,boardTheta,boardHeight,bottomLinkRCoM,bottomLinkMass,bottomLinkTheta,bottomLinkHeight,g,topLinkRCoM,topLinkMass,topLinkTheta);
end