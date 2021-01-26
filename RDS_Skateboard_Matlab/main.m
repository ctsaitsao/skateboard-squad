%% main.m
%
% Description:
%   Application entry point.
%
% Inputs: none
%
% Outputs: none
%
% Notes:
%% Initialize environment
function main 
clear;
close all;
clc;

init_env();

stage_input = input('Enter stage: \n', 's');

while (~strcmp(stage_input, "ramp") && ~strcmp(stage_input, "flat"))
    stage_input = input('Invalid stage, enter again: \n', 's');
end


%% Initialize parameters

params = initial_params;
params.sim.stage = stage_input;
global t_rd
global prevTime_rd
global prevDirection
global i
global prevBottomErrorIdle
global prevTopErrorIdle
global prevBottomErrorPumping
global prevTopErrorPumping
%% Set up events using odeset

options = odeset('Events',@robot_events);
%% Visualize the robot in its initial state

% first five elements are configs (boardX boardY boardTheta bottomLinkTheta topLinkTheta
% last five are velocities corresponding

events = [];
prevBottomErrorIdle = 0;
prevTopErrorIdle = 0;
prevBottomErrorPumping = 0;
prevTopErrorPumping = 0;


t_rd = 0;
prevTime_rd = 0;
prevDirection = 1;


trick = params.sim.trick;


boardX_init = -2;
boardY_init = 2;
boardTheta_init = 0;
bottomLinkTheta_init = 0;
topLinkTheta_init = 0;
boardDX_init = 2; 
boardDY_init = 0;
boardDTheta_init = 0;
bottomLinkDTheta_init = 0;
topLinkDTheta_init = 0;

stage = params.sim.stage;

thetaRamp = asin((params.boardLength/2)/params.trackRadius);
trackLeftS_init = -params.trackRadius*thetaRamp;
trackRightS_init = params.trackRadius*thetaRamp;

    
  

x_IC = [boardX_init; boardY_init; boardTheta_init;...
        bottomLinkTheta_init; topLinkTheta_init; ...
        boardDX_init; boardDY_init; boardDTheta_init;...
        bottomLinkDTheta_init; topLinkDTheta_init];
       
x_IC_plot = x_IC(1:5);

      
switch stage
    
    case 'ramp'
    x_IC = [x_IC; trackLeftS_init; trackRightS_init];
    x_IC_plot = [x_IC_plot; x_IC(11); x_IC(12)];
    
end

i = 0;
 
plot_robot(x_IC_plot, params,'new_fig',false);




%% Simulate the robot forward in time     

% initial conditions
tnow = 0.0;            
% starting time

% start with null matrices for holding results -- we'll be adding to these
% with each segment of the simulation
tsim = [];
xsim = [];
F_list = [];
DX_Matrix = [];
DY_Matrix = [];
DTheta_Matrix = [];
DTheta_bottomlink_Matrix = [];
DTheta_toplink_Matrix = [];
TotEnergy = [];
robotCoM_Matrix = [];
T = [];



% create a place for constraint forces
F = [];

while tnow < params.sim.tfinal

    tspan = [tnow params.sim.tfinal];
    
    tnow
    [tseg, xseg, ~, ~, ie] = ode45(@robot_dynamics, tspan, x_IC, options);

    % augment tsim and xsim; renew ICs
    tsim = [tsim;tseg];
    xsim = [xsim;xseg];
    tnow = tsim(end);
    x_IC = xsim(end,:);
    
    % compute the constraint forces that were active during the jump
    Fseg = zeros(2,length(tseg));
    for ii=1:length(tseg)
        [~,Fseg(:,ii)] = robot_dynamics(tseg(ii),xseg(ii,:)');
         
    end
    
    F_list = [F_list,Fseg];
    
    
    if tseg(end) < params.sim.tfinal  % termination was triggered by an event
        [x_IC] = change_constraints(x_IC,ie);
    end
    
    
    
   
end

%%  Plot Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Energy plots to make sure it's staying conserved
figure;
subplot(2,3,1)
plot(T, DX_Matrix, 'r-', 'LineWidth', 2);
ylabel('DX (m/s)');
xlabel('time (sec)');
subplot(2,3,2)
plot(T, DY_Matrix, 'r-', 'LineWidth', 2);
ylabel('DY (m/s)');
xlabel('time (sec)');
subplot(2,3,3)
plot(T, DTheta_Matrix, 'r-', 'LineWidth', 2);
ylabel('DTheta (rad/s)');
xlabel('time (sec)');
subplot(2,3,4)
plot(T, DTheta_bottomlink_Matrix, 'r-', 'LineWidth', 2);
ylabel('DTheta bottomLink (rad/s)');
xlabel('time (sec)');
subplot(2,3,5)
plot(T, DTheta_toplink_Matrix, 'r-', 'LineWidth', 2);
ylabel('DTheta topLink (rad/s)');
xlabel('time (sec)');
subplot(2,3,6)
plot(T, TotEnergy, 'b-', 'LineWidth', 2);
ylabel('Energy (J)');
xlabel('time (sec)');
hold off

figure;
plot(robotCoM_Matrix(:,1), robotCoM_Matrix(:,2), 'b-', 'LineWidth', 2);
xlabel('robotCoM x');
ylabel('robotCoM y');
hold off

figure;
subplot(1,2,1)
plot(tsim, F_list(1,:), 'r-', 'LineWidth', 2);
ylabel('Left constraint (N)');
xlabel('time (sec)');
subplot(1,2,2)
plot(tsim, F_list(2,:), 'r-', 'LineWidth', 2);
ylabel('Right constraint (N)');
xlabel('time (sec)');
hold off

figure
subplot(2,1,1)
plot(tsim, xsim(:,5), 'LineWidth', 2);
ylabel('Top joint angle');
xlabel('time (sec)');
subplot(2,1,2)
plot(tsim, xsim(:,4), 'LineWidth', 2);
ylabel('Bottom joint angle');
xlabel('time (sec)');
hold off




% Now let's animate

% Let's resample the simulator output so we can animate with evenly-spaced
% points in (time,state).
% 1) deal with possible duplicate times in tsim:
% (https://www.mathworks.com/matlabcentral/answers/321603-how-do-i-interpolate-1d-data-if-i-do-not-have-unique-values
tsim = cumsum(ones(size(tsim)))*eps + tsim;

% 2) resample the duplicate-free time vector:
t_anim = 0:params.sim.dt:tsim(end);

% 3) resample the state-vs-time array:
x_anim = interp1(tsim,xsim,t_anim);
x_anim = x_anim'; % transpose so that xsim is 10xN (N = number of timesteps)

% 4) resample the constraint forces-vs-time array:
F_anim = interp1(tsim,F_list',t_anim);
F_anim = F_anim';

switch stage
    
    case 'ramp'
        xAnim = x_anim([1:5,11,12],:);
    case 'flat'
        xAnim = x_anim(1:5,:);
end

animate_robot(xAnim, F_anim, params,'trace_board_com',true,...
    'trace_bottomLink_com',true,'trace_topLink_com',true,'trace_robot_com',...
     true,'show_constraint_forces',true,'video',true);
fprintf('Done!\n');

%% BELOW HERE ARE THE NESTED FUNCTIONS, ROBOT_DYNAMICS AND ROBOT_EVENTS

%% THEY HAVE ACCESS TO ALL VARIABLES IN MAIN

%% robot_dynamics.m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Description:
%   Computes the constraint forces: 
%       Fnow = inv(A*Minv*A')*(A*Minv*(Q-H) + Adotqdot)
%
%   Also computes the derivative of the state:
%       x_dot(1:5) = (I - A'*inv(A*A')*A)*x(6:10)
%       x_dot(6:10) = inv(M)*(Q - H - A'F)
%
% Inputs:
%   t: time (scalar)
%   x: the 10x1 state vector
%   params: a struct with many elements, generated by calling init_params.m
%
% Outputs:
%   dx: derivative of state x with respect to time.
%   energy: total energy of the system at state x

function [dx, F] = robot_dynamics(t,x)

    
switch stage
    case 'ramp'
if t_rd <= 0.1
    direction = prevDirection;      
elseif t_rd > 0.1
    if x(3) > 0 && x(8) > 0
        direction = 1;
    elseif x(3) < 0 && x(8) < 0
        direction = 1;
    elseif x(3) > 0 && x(8) < 0
        direction = 0;
    else
        direction = 0;
    end
    
    if direction ~= prevDirection
        t_rd = 0;
        prevDirection = direction;
    end  
    
end
end

t_rd = t_rd + (t-prevTime_rd);
prevTime_rd = t;


switch stage
    
    case 'ramp'
        
    dx = zeros(numel(x),1);
    nq = numel(x)/2-1;    % assume that x = [q;q_dot; s(1); s(2)];
    q_dot = x(nq+1:2*nq);
    s = x(2*nq+1:2*nq+2);

    case 'flat'
        
    dx = zeros(numel(x),1);
    nq = numel(x)/2;    % assume that x = [q;q_dot; s(1); s(2)];
    q_dot = x(nq+1:2*nq);   

end


% solve for control inputs at this instant


%% DECIDE WHAT Q IS  

% inputs for feedback control: board Current Angle, bottomLink Current Angle,
% bottomLink Desired Angle, topLink Current Angle, topLink Desired Angle

switch trick
    
    case 'pumping'
   
[bottomMotorTorque, topMotorTorque] = pid_pumping(x(4), x(5), direction);

    case 'manual'

[bottomMotorTorque, topMotorTorque] = pid_manual(x(4), x(5));

    case 'idle'
        
[bottomMotorTorque, topMotorTorque] = pid_idle(x(4), x(5));


end

Q = [0; 0; 0; bottomMotorTorque; topMotorTorque];


%%

% find the parts that don't depend on constraint forces
H = H_eom(x,params);
Minv = inv_mass_matrix(x,params);

switch stage
    
    case 'ramp'
        
        [A,Hessian] = constraint_derivatives_ramp(x,params);
        
    case 'flat'
        [A, Hessian] = constraint_derivatives_flat(x,params);
end

% compute energy

TE_now = totalEnergy(x, params);
DX_Matrix = [DX_Matrix; x(6)]; % matrix keeping track of DX for skateboard
DY_Matrix = [DY_Matrix; x(7)]; % matrix keeping track of DY for skateboard
DTheta_Matrix = [DTheta_Matrix; x(8)]; % matrix keeping track of DTheta for skateboard
DTheta_bottomlink_Matrix = [DTheta_bottomlink_Matrix; x(9)];
DTheta_toplink_Matrix = [DTheta_toplink_Matrix; x(10)];
TotEnergy = [TotEnergy; TE_now]; % matrix keeping track of total energy
fkins = fwd_kin(x, params);
robotCoM_Matrix = [robotCoM_Matrix; fkins(1,4), fkins(2,4)];
T = [T; t]; % matrix keeping track of time

n_active_constraints = sum(params.sim.constraints);
F = zeros(2,1);

if n_active_constraints == 0  % if there are no constraints active, then there are no forces
    
    dx(1:nq) = q_dot;
    dx(nq+1:2*nq) = Minv*(Q - H);
    
else  % if there are constraints active, we must compute the constraint forces
    A = A(params.sim.constraints,:);   % eliminate inactive rows
    Hessian_active = Hessian(:,:,params.sim.constraints);  % eliminate inactive hessians
    Adotqdot = zeros(n_active_constraints,1);
    for ic=1:n_active_constraints
        Adotqdot(ic) = q_dot'*Hessian_active(:,:,ic)*q_dot;
    end
    
    % compute the constraint forces and accelerations
    F_active = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);   % these are the constraint forces
    dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(nq+1:2*nq);
    dx(nq+1:2*nq) = Minv*(Q - H - A'*F_active) - (A'*((A*A')\A)*x(nq+1:2*nq)).*1000;

    % 4x1 vector of constraint forces
    F(params.sim.constraints) = F_active;   

end

switch stage
    
    case 'ramp'

    [leftWheelPos,rightWheelPos] = wheel_coordinates(x,params);
    [leftWheelVelo,rightWheelVelo] = wheel_velocities(x,params);
    [p_l,t_l,~] = track(s(1),params);
    [p_r,t_r,~] = track(s(2),params);

    dx(2*nq+1) = (leftWheelVelo + params.sim.gain*(leftWheelPos - p_l))'*t_l;
    dx(2*nq+2) = (rightWheelVelo + params.sim.gain*(rightWheelPos - p_r))'*t_r;
    
end

switch stage
    
    case 'ramp'

C = constraints_ramp(x,params);
events = F;
events(~params.sim.constraints) = -C(~params.sim.constraints);

    case 'flat'
        
C = constraints_flat(x,params);
events = F;
events(~params.sim.constraints) = -C(~params.sim.constraints);
 

end

end

%% end of robot_dynamics.m


%% Event function for ODE45 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Description:
%   Event function that is called when a constraint becomes inactive (or, in the future, active) 
%
% Inputs:
%   t and x are required, but not used
%   F is shared with parent function
%
% Outputs:
%   value
%   isterminal
%   direction
function [value,isterminal,direction] = robot_events(~,~)

   value = events;
   isterminal = ones(2,1);
   direction = -ones(2,1);

end
%% end of robot_events.m 

%% change_constraints.m
%
% Description:
%   function to handle changes in constraints, depending on the current
%   status of params.sim.constraints as well as ie, the index of the last
%   event to occur
%
% Inputs:
%   x_IC: the current state of the robot, which will be the initial
%   conditions for the next segment of integration
%   ie: the index of events returned by robot_events.m
%
% Outputs:
%   x_IC: the current state of the robot, which might be updated if the
%   event that occurred was a collision

function [x_IC] = change_constraints(x_IC,ie)

A = [];
restitution = [];
collision = 0;



for i1 = 1:length(ie)  % I'm not sure if ie is ever a vector ... just being sure!
    if params.sim.constraints(ie(i1)) == 1  % if the event came from an active constraint
        params.sim.constraints(ie(i1)) = 0; % then make the constraint inactive
    else    % the event came from an inactive constraint --> collision
        collision = 1;
        params.sim.constraints(ie(i1)) = 1; % make the constraint active
        % find the constraint jacobian
        
            switch stage
    
             case 'ramp'
             [A_all,~] = constraint_derivatives_ramp(x_IC,params);
             
             case 'flat'  
             [A_all,~] = constraint_derivatives_flat(x_IC,params);
            end
            
       A = [A;A_all(ie(i1),:)];
       restitution = [restitution,1+params.sim.restitution(ie(i1))];
    end
end

if collision == 1
    Minv = inv_mass_matrix(x_IC,params);
    % compute the change in velocity due to collision impulses
    x_IC(6:10) = x_IC(6:10) - (Minv*A'*inv(A*Minv*A')*diag(restitution)*A*x_IC(6:10)')';
    % Often in a collision, the constraint forces will be violated
    % immediately, rendering event detection useless since it requires a
    % smoothly changing variable.  Therefore, we need to check the
    % constraint forces and turn them off if they act in the wrong
    % direction
    [~,F] = robot_dynamics(0,x_IC');
    for i1=1:2
        if F(i1)<10^-6, params.sim.constraints(i1) = 0; end  % turn off unilateral constraints with negative forces
    end
end



end
%% end of change_constraints.m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% End of main.m
end