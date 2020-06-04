%% constraints_ramp_lip.m
%
% Description:
%   Wrapper function for autogen_constraints_ramp_lip.m
%   Computes the constraint forces of the robot.
%
% Inputs:
%   x: the state vector, x = [q; q_dot];
%   params: a struct with many elements, generated by calling init_params.m
%
% Outputs:
%   C_all = values of two constraint equations

function [C_all] = constraints_ramp_lip(x,params)



% autogen inputs:
% boardY,boardTheta,boardHeight,boardLength,wheelRadius

C_all = autogen_constraints_ramp_lip(x(2), x(3), params.boardHeight,...
                                 params.boardLength, params.wheelRadius);

end

