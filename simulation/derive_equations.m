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

Q = [frictForceX; frictForceY; 0; bottomMotorTorque; topMotorTorque];
% SET FRICTX, FRICTY == 0 TO START

frictForceX = 0;
frictForceY= 0;

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
syms constL constR real

boardLeftX =  boardX + boardHeight/2 * sin(boardTheta) - boardLength/2 * cos(boardTheta) + wheelRadius * sin(boardTheta);

boardLeftY = boardY - boardHeight/2 * cos(boardTheta)  - boardLength/2 * sin(boardTheta) - wheelRadius * cos(boardTheta);

boardRightX = boardX + boardHeight/2 * sin(boardTheta) + boardLength/2 * cos(boardTheta) + wheelRadius * sin(boardTheta) ;

boardRightY = boardY - boardHeight/2 * cos(boardTheta) + boardLength/2 * sin(boardTheta) - wheelRadius * cos(boardTheta);


FK_leftWheel = [boardLeftX;boardLeftY];
FK_rightWheel = [boardRightX;boardRightY];

constL = (boardLeftX - pLeftX)*normLeftX + (boardLeftY - pLeftY)*normLeftY;
constR = (boardRightX - pRightX)*normRightX + (boardRightY - pRightY)*normRightY;

constraints = [constL; constR];

matlabFunction(FK_leftWheel,FK_rightWheel,'File','autogen_fk_wheels');
matlabFunction(constraints,'File','autogen_constraints');

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


syms A_all H_constL H_constR real

A_all = jacobian(constraints, q);

H_constL = hessian(constL, q);
H_constR = hessian(constR, q);

matlabFunction(leftWheelDFK,rightWheelDFK,'File','autogen_wheel_velocities');
matlabFunction(bottomLinkDXCoM, bottomLinkDYCoM, topLinkDXCoM, topLinkDYCoM, 'File', 'autogen_CoM_velocities');
matlabFunction(A_all, H_constL, H_constR, 'File', 'autogen_constraint_derivatives');

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

% %% Generate (nonlinear) state-space model from manipulator equation
% fprintf('\tConverting manipulator equation to state-space form...\n');
% 
% f_ss = sym('f_ss',[2*numel(q),1],'real'); % drift vector field
% g_ss = sym('g_ss',[2*numel(q),2],'real'); % control vector field
% 
% % Note: building the state-space dynamics from the manipulator equation has
% % two steps:
% %
% %   1) the manipulator equation gives the *inverse* dynamics (mapping from
% %   state and accelerations to forces/torques). We need
% %   the *forward* dynamics (mapping from state and forces/torques to
% %   accelerations), so we must solve the manipulator equation for ddq:
% %
% %       ddq = -Minv*(C*dq + G) + Minv * (S * u), S as [5x2]
% %
% %   We could write a generalization of this equation as follows:
% %
% %       ddq = temp_drift + temp_ctrl * u
% %
% %   where
% %       temp_drift = -Minv*(C*dq + G) and
% %       temp_ctrl  = Minv * S 
% %   The motivation for these variable names (temp_drift & temp_ctrl) is
% %   explained below.
% %
% %   2) Solving the manipular equation for ddq gives us the forward
% %   dynamics, represented as a system of 2nd-order ODEs. The state-space
% %   dynamics are represented as a system of 1st-order ODEs, so we must
% %   convert the 2nd-order system to a 1st-order system:
% %
% %       x   = [q;  dq]; % state vector
% %       dx  = [dq; ddq]; % derivative of the state vector w.r.t. time
% %
% %       % Substitute the forward dynamics for ddq in the equation for dx,
% %       resulting in a system of 4 1st-order ODEs:
% %
% %       dx = [dq; temp_drift + temp_ctrl*u];
% %
% %   There is more structure to this system of ODEs than may be
% %   apparent at first glance. In particular, the control input u appears
% %   linearly in the state-space dynamics, so we can (abstractly) write
% %   the state-space dynamics as
% %
% %       dx = f_ss(x) + g_ss(x)*u,
% %
% %   where
% %       f_ss(x) = [0; 0; temp_drift(1); temp_drift(2)] and
% %       g_ss(x) = [0; 0; temp_ctrl(1);  temp_ctrl(2)].
% %
% %   This is called a "control-affine" system because the dynamics are
% %   "affine" (linear) in the control input u; g_ss(x) (the coefficient of
% %   u) is called the "control vector field", and f_ss(x) (everything else
% %   that is not a coefficient of u) is called the "drift vector field"
% %   (because it describes how the state "drifts" in the absence of control
% %   input).
% %
% %   Note: the "_ss" suffix stands for "state-space" (not "steady-state").
% 
% 
% % Selection Matrix: two controls (one for each motor)
% 
% 
% S = [0 0;...
%      0 0;...
%      0 0;...
%      1 0;...
%      0 1];
% 
% 
% temp_drift = simplify(-Minv * (C*dq + G)); % stuff that does not care about u
% temp_ctrl = simplify(Minv * S); % stuff that does care about u
% 
% % Build state-space representation:
% for i = 1:numel(q)
%     f_ss(i)   = dq(i);
%     g_ss(i,:)   = 0;
%     f_ss(i+numel(q)) = temp_drift(i);
%     g_ss(i+numel(q),:) = temp_ctrl(i,:);
% end
% 
% matlabFunction(f_ss,'File','autogen_drift_vector_field');
% matlabFunction(g_ss,'File','autogen_control_vector_field');
% 
% fprintf('\t...done converting manipulator equation to state-space form.\n');
% 
% %% Linearize the state-space model around the upright equilibrium
% fprintf('\tLinearizing the dynamics about the upright equilibrium...\n')
% 
% % Note: we linearize the state-space dynamics by performing a 1st-order
% % Taylor series approximation, evaluated at the upright equilibrium:
% boardXEq = 0;
% boardDXEq = 0;
% 
% boardYEq = 0;
% boardDYEq = 0;
% 
% boardThetaEq = 0;
% boardDThetaEq = 0;
% 
% bottomLinkThetaEq = 0;
% bottomLinkDThetaEq = 0;
% 
% topLinkThetaEq = 0;
% topLinkDThetaEq = 0;
% 
% 
% % Note: we have been talking about x for a while above but haven't needed
% % to define it as a symbolic variable until now:
% x = [q; dq]; % just makes the code below easier to read)
% 
% % define the equilibrium state:
% x_eq = [boardXEq; boardDXEq; boardYEq; boardDYEq; boardThetaEq; boardDThetaEq;...
%         bottomLinkThetaEq; bottomLinkDThetaEq; topLinkThetaEq; topLinkDThetaEq];
% 
% % Step 1 of Taylor series approximation is to compute Jacobians:
% f_ss_jac = jacobian(f_ss, x);
% g_ss_jac = g_ss;
% % Note: g_ss_jac = g_ss because the nonlinear state-space dynamics are
% % control-affine, but you should verify this!
% 
% % Evaluate Jacobians at the upright equilibrium:
% A = simplify(subs(f_ss_jac, x, x_eq));
% B = simplify(subs(g_ss_jac, x, x_eq));
% 
% fprintf('\t...done linearizing the dynamics about the upright equilibrium.\n')
% 
% matlabFunction(A,'File','autogen_upright_state_matrix');
% matlabFunction(B,'File','autogen_upright_input_matrix');
% 
% %% Compute the controllability matrix and its rank
% fprintf('\tComputing the controllability matrix of the linearized model...\n')
% 
% % Note: the linearized dynamics "near" the upright equilibrium are 
% % controllable if rank(Co) = dim(x) (4 in our cart-pendulum example). This
% % is called the Kalman controllability rank condition and is an
% % easy-to-compute metric of controllability for a linear system, i.e., a 
% % useful sanity check.
% 
% % Note: our cart-pendulum is underactuated (1 control input, 2 degees of
% % freedom). Does that mean it is not controllable?
% 
% % Co = [];
% % for i = 1:((2*numel(q)))
% %     Co = horzcat(Co,simplify((A^(i-1))*B));
% % end
% % 
% % fprintf('\t...done. The controllability matrix of the linearized model has rank %d.\n',...
% %     rank(Co));
% 
% %% Done!
% fprintf('...done deriving cart-pendulum equations.\n');
end