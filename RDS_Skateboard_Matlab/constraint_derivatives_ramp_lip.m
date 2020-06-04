%% constraint_derivatives.m
%
% Description:
%   Wrapper function for autogen_constraint_derivatives.m
%   Computes the constraint jacobian and hessians for the jumping robot.
%
% Inputs:
%   x: the state vector, x = [q; q_dot];
%   params: a struct with many elements, generated by calling init_params.m
%
% Outputs:
%   A_all: a 3x3 jacobian of constraint equation derivatives.  If only a
%   subset of constraints are active, then only those rows of A_all will be
%   used to compute the A matrix for that situation.
%
%   H_c1, H_c2, H_c3:  the hessian matrices; one for each constraint.  Note
%   that H_c3 is the null matrix, but we keep it for clean, robust code.

function [A_all,Hessian] = constraint_derivatives_ramp_lip(x,params)





% autogen inputs: boardTheta,boardHeight,boardLength,wheelRadius

[A_all,H_constL,H_constR] = autogen_constraint_derivatives_ramp_lip(...
                             x(3), params.boardHeight, params.boardLength,...
                             params.wheelRadius);
                               
                 
Hessian = cat(3,H_constL,H_constR);

end

