%% derive_equations.m
%
% Description:
%   Symbolically derive equations related to the skateboarding robot, including
%   the full nonlinear state-space dynamics and the state-space dynamics of
%   the system linearized about the unstable upright equilibrium.
%
% Inputs:
%   none
%
% Outputs:
%   none

function derive_equations
clear;
close all;
clc;

fprintf('Deriving skateboarding robot equations...\n');

%% Define generalized variables
fprintf('\tInitializing generalized coordinates, velocities, accelerations, and forces...\n');

% Generalized coordinates:
% boardX: position of the skateboard CoM along the x-axis
% boardY: position of the skateboard CoM along the y-axis
% boardTheta: angle of the skateboard CoM wrt the x-axis
% bottomLinkTheta: angle of the bottom link, ccw+ w.r.t. skateboard angle
% topLinkTheta: angle of the pendulum, ccw+ w.r.t. top link angle

syms boardX boardY boardTheta bottomLinkTheta topLinkTheta real
q = [boardX; boardY; boardTheta; bottomLinkTheta; topLinkTheta];

% Generalized velocities:
% boardDX: velocity of the skateboard CoM along the x-axis
% boardDY: velocity of the skateboard CoM along the y-axis
% boardDTheta: rate of change of angle of the skateboard CoM wrt the x-axis
% bottomLinkDTheta: angular velocity of angle of the bottom link, ccw+ w.r.t. skateboard angle
% topLinkDTheta: angular velocity of angle of the pendulum, ccw+ w.r.t. top link angle
syms boardDX boardDY boardDTheta bottomLinkDTheta topLinkDTheta real
dq = [boardDX; boardDY; boardDTheta; bottomLinkDTheta; topLinkDTheta];

% Generalized accelerations:
syms boardDDX boardDDY boardDDTheta bottomLinkDDTheta topLinkDDTheta real
% boardDX: acceleration of the skateboard CoM along the x-axis
% boardDY: acceleration of the skateboard CoM along the y-axis
% boardDTheta: angular acceleration of angle of the skateboard CoM wrt the x-axis
% bottomLinkDTheta: angular acceleration of angle of the bottom link, ccw+ w.r.t. skateboard angle
% topLinkDTheta: angular acceleration of angle of the pendulum, ccw+ w.r.t. top link angle
ddq = [boardDDX; boardDDY; boardDDTheta; bottomLinkDDTheta; topLinkDDTheta];

% Generalized forces:

syms  frictForceX frictForceY bottomMotorTorque topMotorTorque  real % force on the cart, in the x-direction
% Note: we are not defining a torque at the cart-pendulum joint, so the
% system is underactuated. This will make things interesting!

Q = [0; 0; 0; bottomMotorTorque; topMotorTorque];


fprintf('\t...done.\n');


%% Define kinematics variables
fprintf('\tInitializing kinematics variables...\n');

% coordinate of the bottom link CoM:
syms bottomLinkXCoM bottomLinkYCoM  real
% Note: we already defined x_cart as the x-coordinate of the cart CoM.

% coordinates of the top link CoM:
syms topLinkXCoM topLinkYCoM  real

% Dimensions: distance from bottom, top edge to CoM bottom link:
syms bottomLinkRCoM topLinkRCoM real

% Dimensions: length of the bottom, top link:
syms bottomLinkHeight topLinkHeight real

% Location of the overall center of mass of the robot
syms robotXCoM robotYCoM real

% Geometry
syms boardLength boardHeight wheelRadius real

% Note: the dynamics don't care where the pendulum tip is (only the 
% pendulum CoM), but keeping track of the tip will be helpful for 
% visualizing the robot (could also be helpful for some control tasks).
fprintf('\t...done.\n');

%% Define inertial (and other) parameters
fprintf('\tInitializing inertial (and other) parameters...\n');

% Mass of each body:
syms boardMass bottomLinkMass topLinkMass real

% Rotational inertia of the pendulum:
syms boardI bottomLinkI topLinkI real
% Note: the cart cannot rotate so we don't care about its rotational
% inertia

% Viscous damping at each joint: (add enough that it doesnt flail like
% crazy)

% Note: we'll account for damping at each joint (ground-cart and 
% cart-pendulum) but we can set each damping constant to 0 later if we
% want.

% Other variables
g = sym('g','real'); % gravity
t = sym('t','real'); % time

fprintf('\t...done.\n');

%% Forward kinematics
fprintf('\tGenerating forward kinematics equations...\n');

% compute the (x,y) location of the pendulum's center of mass:

boardY = (boardY + wheelRadius + boardHeight/2);

bottomLinkXCoM = boardX + boardHeight/2 * sin(boardTheta + pi) + bottomLinkRCoM * sin(boardTheta + bottomLinkTheta + pi); % TODO
bottomLinkYCoM = boardY + boardHeight/2 * cos(boardTheta) + bottomLinkRCoM * cos(boardTheta + bottomLinkTheta); % TODO

topLinkXCoM = boardX + boardHeight/2 * sin(boardTheta + pi) + bottomLinkHeight * sin(boardTheta + bottomLinkTheta + pi) + topLinkRCoM * sin(boardTheta + bottomLinkTheta + topLinkTheta + pi); 
topLinkYCoM = boardY + boardHeight/2 * cos(boardTheta) + bottomLinkHeight * cos(boardTheta + bottomLinkTheta) + topLinkRCoM * cos(boardTheta + bottomLinkTheta + topLinkTheta);

% compute the (x,y) location of the robot's tip:
robotTipX = boardX + boardHeight/2 * sin(boardTheta + pi) + bottomLinkHeight * sin(boardTheta + bottomLinkTheta + pi) + topLinkHeight * sin(boardTheta + bottomLinkTheta + topLinkTheta + pi); 
robotTipY = boardY + boardHeight/2 * cos(boardTheta) + bottomLinkHeight * cos(boardTheta + bottomLinkTheta) + topLinkHeight * cos(boardTheta + bottomLinkTheta + topLinkTheta);

% robot's CoM

robotXCoM = (boardX * boardMass + bottomLinkXCoM * bottomLinkMass + topLinkXCoM * topLinkMass)/(boardMass + bottomLinkMass + topLinkMass);
robotYCoM = (boardY * boardMass + bottomLinkYCoM * bottomLinkMass + topLinkYCoM * topLinkMass)/(boardMass + bottomLinkMass + topLinkMass);

% create a 2x5 array to hold all forward kinematics (FK) outputs:
FK = [boardX, bottomLinkXCoM, topLinkXCoM, robotXCoM, robotTipX;
      boardY, bottomLinkYCoM, topLinkYCoM, robotYCoM, robotTipY];

% generate a MATLAB function to compute all the FK outputs:
matlabFunction(FK,'File','autogen_fwd_kin');
% Note: this autogenerated function will make plotting/animation easier
% later. Could also help with some control tasks.
fprintf('\t...done.\n');

%% Constraint equations

fprintf("Initializing constraint variables...\n");

syms boardLeftX boardLeftY boardRightX boardRightY real
syms pLeftX pLeftY pRightX pRightY normLeftX normLeftY normRightX normRightY real
syms constL_ramp constR_ramp constL_flat constR_flat real

boardLeftX =  boardX + boardHeight/2 * sin(boardTheta) - boardLength/2 * cos(boardTheta) + wheelRadius * sin(boardTheta);

boardLeftY = boardY - boardHeight/2 * cos(boardTheta)  - boardLength/2 * sin(boardTheta) - wheelRadius * cos(boardTheta);

boardRightX = boardX + boardHeight/2 * sin(boardTheta) + boardLength/2 * cos(boardTheta) + wheelRadius * sin(boardTheta) ;

boardRightY = boardY - boardHeight/2 * cos(boardTheta) + boardLength/2 * sin(boardTheta) - wheelRadius * cos(boardTheta);


FK_leftWheel = [boardLeftX;boardLeftY];
FK_rightWheel = [boardRightX;boardRightY];

constL_flat = -boardLeftY;
constR_flat = -boardRightY;

constL_ramp = (boardLeftX - pLeftX)*normLeftX + (boardLeftY - pLeftY)*normLeftY;
constR_ramp = (boardRightX - pRightX)*normRightX + (boardRightY - pRightY)*normRightY;

constraints_flat = [constL_flat; constR_flat];
constraints_ramp = [constL_ramp; constR_ramp];



matlabFunction(FK_leftWheel,FK_rightWheel,'File','autogen_fk_wheels');
matlabFunction(constraints_flat,'File','autogen_constraints_flat');
matlabFunction(constraints_ramp,'File','autogen_constraints_ramp');


%% Derivatives
fprintf('\tGenerating time derivatives of the kinematics equations...\n');
% Neat trick to compute derivatives using the chain rule
% from https://github.com/MatthewPeterKelly/OptimTraj/blob/master/demo/fiveLinkBiped/Derive_Equations.m
derivative = @(in)( jacobian(in,[q;dq])*[dq;ddq] );

% CoM velocities:
syms boardDXCoM boardDYCoM bottomLinkDXCoM bottomLinkDYCoM topLinkDXCoM topLinkDYCoM real
syms leftWheelDFK rightWheelDFK real

% MAYBE NECCESSARY MAYBE NOT
boardDX = derivative(boardX); 
boardDY = derivative(boardY);

bottomLinkDXCoM = derivative(bottomLinkXCoM);
bottomLinkDYCoM = derivative(bottomLinkYCoM);


topLinkDXCoM = derivative(topLinkXCoM);
topLinkDYCoM = derivative(topLinkYCoM);


boardDTheta = derivative(boardTheta);
bottomLinkDTheta = derivative(bottomLinkTheta);
topLinkDTheta = derivative(topLinkTheta);


leftWheelDFK = jacobian(FK_leftWheel,q)*dq;
rightWheelDFK = jacobian(FK_rightWheel,q)*dq;


syms A_all_ramp H_constL_ramp H_constR_ramp real
syms A_all_flat H_constL_flat H_constR_flat real

A_all_ramp = jacobian(constraints_ramp, q);
A_all_flat = jacobian(constraints_flat, q);

H_constL_ramp = hessian(constL_ramp, q);
H_constR_ramp = hessian(constR_ramp, q);

H_constL_flat = hessian(constL_flat, q);
H_constR_flat = hessian(constR_flat, q);


matlabFunction(leftWheelDFK,rightWheelDFK,'File','autogen_wheel_velocities');
matlabFunction(bottomLinkDXCoM, bottomLinkDYCoM, topLinkDXCoM, topLinkDYCoM, 'File', 'autogen_CoM_velocities');
matlabFunction(A_all_ramp, H_constL_ramp, H_constR_ramp, 'File', 'autogen_constraint_derivatives_ramp');
matlabFunction(A_all_flat, H_constL_flat, H_constR_flat, 'File', 'autogen_constraint_derivatives_flat');

fprintf('\t...done with constraint equations.\n');

%% Kinetic energy
fprintf('\tGenerating kinetic energy equation...\n');

syms boardKE bottomLinkKE topLinkKE KE real

% kinetic energy of each body ONLY TRANSLATION:
boardKE = 1/2 * boardMass * (boardDX^2+boardDY^2) + (1/2 * boardI * boardDTheta^2);
bottomLinkKE = (1/2* bottomLinkMass * ((bottomLinkDXCoM)^2+(bottomLinkDYCoM)^2)) + (1/2 * bottomLinkI * (boardDTheta + bottomLinkDTheta)^2);
topLinkKE = (1/2* topLinkMass * ((topLinkDXCoM)^2+(topLinkDYCoM)^2)) + (1/2 * topLinkI * (boardDTheta + bottomLinkDTheta + topLinkDTheta)^2);

% total kinetic energy:
KE = boardKE + bottomLinkKE + topLinkKE;

fprintf('\t...done.\n');

%% Potential energy
fprintf('\tGenerating potential energy equation...\n');

syms boardPE bottomLinkPE topLinkPE PE real

% potential energy of the pendulum:


PE = robotYCoM * g * (boardMass + bottomLinkMass + topLinkMass);
% Note: the cart moves horizontally so its gravitational PE never changes.

fprintf('\t...done.\n');

syms totEnergy real

totEnergy = PE + KE;
matlabFunction(totEnergy,'File','autogen_total_energy');

%% Lagrangian
fprintf('\tGenerating Lagrangian...\n');

syms L real
L = KE - PE;

fprintf('\t...done.\n');

%% Euler-Lagrange equations
fprintf('\tGenerating Euler-Lagrange equations of motion...\n')

% Note: the Euler-Lagrange equations of motion are
%   d(del L/del dq)/dt - (del L/del qa) = generalized forces;
% I did not include damping in my Lagrangian (although it can be done, at
% least for viscous damping) so I have to lump it in with the other
% generalized forces (in this case, the only other generalized force is the
% control input u, i.e. the force applied horizontally to the cart).


% Variable initializations:


del_L_del_dq = sym('del_L_del_dq',[numel(q),1],'real'); % del L/del q_dot
ELeq_term1  = sym('ELeq_term1',[numel(q),1],'real'); % d(del L/del dq)/dt
ELeq_term2  = sym('ELeq_term2',[numel(q),1],'real'); % del L/del q

del_L_del_dq = jacobian(KE,dq)';
ELeq_term1 = simplify(jacobian(del_L_del_dq,[q;dq])*[dq;ddq]);
ELeq_term2 = simplify(jacobian((L),q))';
% Note: no constraint forces yet

fprintf('\t...done.\n')
fprintf('\tSolving for the mass matrix...\n');

M = sym('M',[numel(q),numel(q)],'real');
H = sym('H',[numel(q),1],'real');

M = simplify(jacobian(del_L_del_dq,dq));
H = simplify(jacobian(del_L_del_dq,q)*dq - ELeq_term2);

fprintf('\t\t...done building M and H.\n');


% compute and store inv(M):
fprintf('\t\tComputing inv(M)...\n')
Minv = sym('Minv',[numel(q),numel(q)],'real');
Minv = simplify(inv(M));
fprintf('\t\t...done.\n')

fprintf('\t\tGenerating MATLAB functions...\n');
matlabFunction(M,'File','autogen_mass_matrix');
fprintf('\t\t...done building M function\n');
matlabFunction(Minv,'File','autogen_inverse_mass_matrix');
fprintf('\t\t...done building Minv function\n');
matlabFunction(H,'File','autogen_H_eom');
fprintf('\t\t...done.\n');

fprintf('...done deriving cart-pendulum equations.\n');


end