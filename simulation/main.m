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
events = [];
trick = params.sim.trick;

%% Set max time step using odeset
options = odeset('MaxStep',params.sim.dt);

%% Set up events using odeset
options = odeset('Events',@robot_events);

%% [FLAT CASE] Set the initial equilibrium pose of the robot 
x_eq = zeros(10,1);  % [q, dq] = [boardX; boardY; boardTheta; bottomLinkTheta; topLinkTheta; boardDX; boardDY; boardDTheta; bottomLinkDTheta; topLinkDTheta];
x_eq(3) = deg2rad(15);
x_eq(4) = deg2rad(55);
x_eq(5) = eq_top_link_angle(x_eq(3), x_eq(4), params);

%% [FLAT CASE] Show the robot in equilibrium
x_eq_plot = x_eq(1:5);

thetaRamp = asin((params.boardLength/2)/params.trackRadius);
trackLeftS_init = -params.trackRadius*thetaRamp;
trackRightS_init = params.trackRadius*thetaRamp;
stage = params.sim.stage;

switch stage
    
    case 'ramp'
    x_eq = [x_eq; trackLeftS_init; trackRightS_init];
    x_eq_plot = [x_eq_plot; x_eq(11); x_eq(12)];
    
end

plot_robot(x_eq_plot,params,'new_fig',false);

%% [FLAT CASE] Set equilibrium motor torques

x_eq_3DOF = [x_eq(3);x_eq(4);x_eq(5);x_eq(8);x_eq(9);x_eq(10)];
G = conservative_forces_3DOF(x_eq_3DOF,params)
% fprintf('%.10f',G(1))
u_eq = [G(2);G(3)];              % initial command
% u_eq = reshape(eq_torques(x_eq(3),x_eq(4),x_eq(5),params),[2,1]);   % outputs same as u_eq = [G(2);G(3)]   
u = u_eq;

%% [FLAT CASE] Get 3DOF LQR gains

[Gains, Poles] = lqr_gains(x_eq_3DOF,params)

%% [FLAT CASE] initialize the state
x_IC = x_eq;
x_IC(6) = 30;  % initial velocity in x direction
% perturb slightly so that something happens  CAN CHANGE TO SMTH ELSE
% x_IC(3) = x_IC(3) + (1/5)*pi;

%% [FLAT CASE] initialize controller memory
memory.u_eq = u_eq;  % not really memory ... just passing the equilibrium 
memory.x_eq = x_eq;  % not really memory ... just passing the equilibrium
memory.y = [x_IC(4:5);x_IC(8)];  % y = stuff you can control 
memory.boardTheta = x_IC(3);  % perfect initial knowledge!

%% [FLAT CASE] Simulate the robot forward in time (main loop)

% initial conditions
tnow = 0.0;            
% starting time

% start with null matrices for holding results -- we'll be adding to these
% with each segment of the simulation
tsim = [];
xsim = [];
F_list = [];
usim = [];
DX_Matrix = [];
DY_Matrix = [];
DTheta_Matrix = [];
DTheta_bottomlink_Matrix = [];
DTheta_toplink_Matrix = [];
TotEnergy = [];
robotCoM_Matrix = [];
T = [];

tcontrol = [];

% create a place for constraint forces
F = [];
dt = params.sim.dt;

while tnow < params.sim.tfinal
    
%     tnow

%     tspan = [tnow params.sim.tfinal];
%     [tseg, xseg, ~, ~, ie] = ode45(@robot_dynamics, tspan, x_IC, options);

    tspan = [tnow, tnow+dt];
    [tseg, xseg] = ode45(@(t,x) robot_dynamics(t,x,u), tspan, x_IC);
    
    % find the state and sensor measurements at the time a read was made
    t_read = tseg(end) - params.sim.delay;
    x_read = interp1(tseg,xseg,t_read);
    y = sensor(x_read);
    
    % compute the control 
    [u,memory] = digital_controller(y,Gains,memory);
    
    % update t_write and x_IC for next iteration
    tnow = tseg(end);
    x_IC = xseg(end,:); 
    
    % augment tsim and xsim; renew ICs
    tsim = [tsim;tseg];
    xsim = [xsim;xseg];
%     tnow = tsim(end);
%     x_IC = xsim(end,:);
    
    % variables based off of tcontrol, not tsim
    tcontrol = [tcontrol;t_read];
    usim = [usim;u];
    
    % compute the constraint forces that were active during the jump
    Fseg = zeros(2,length(tseg));
    for ii=1:length(tseg)
        [~,Fseg(:,ii)] = robot_dynamics(tseg(ii),xseg(ii,:)',u);
%         [~,Fseg(:,ii)] = robot_dynamics(tseg(ii),xseg(ii,:)');         
    end
    
    F_list = [F_list,Fseg];
    
    
%     if tseg(end) < params.sim.tfinal  % termination was triggered by an event
%         [x_IC] = change_constraints(x_IC,ie);
%     end
    
    
end

%%  Plot Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Energy plots to make sure it's staying conserved
% figure;
% subplot(2,3,1)
% plot(T, DX_Matrix, 'r-', 'LineWidth', 2);
% ylabel('DX (m/s)');
% xlabel('time (sec)');
% subplot(2,3,2)
% plot(T, DY_Matrix, 'r-', 'LineWidth', 2);
% ylabel('DY (m/s)');
% xlabel('time (sec)');
% subplot(2,3,3)
% plot(T, DTheta_Matrix, 'r-', 'LineWidth', 2);
% ylabel('DTheta (rad/s)');
% xlabel('time (sec)');
% subplot(2,3,4)
% plot(T, DTheta_bottomlink_Matrix, 'r-', 'LineWidth', 2);
% ylabel('DTheta bottomLink (rad/s)');
% xlabel('time (sec)');
% subplot(2,3,5)
% plot(T, DTheta_toplink_Matrix, 'r-', 'LineWidth', 2);
% ylabel('DTheta topLink (rad/s)');
% xlabel('time (sec)');
% subplot(2,3,6)
% plot(T, TotEnergy, 'b-', 'LineWidth', 2);
% ylabel('Energy (J)');
% xlabel('time (sec)');
% hold off
% 
% figure;
% plot(robotCoM_Matrix(:,1), robotCoM_Matrix(:,2), 'b-', 'LineWidth', 2);
% xlabel('robotCoM x');
% ylabel('robotCoM y');
% hold off
% 
% figure;
% subplot(1,2,1)
% plot(tsim, F_list(1,:), 'r-', 'LineWidth', 2);
% ylabel('Left constraint (N)');
% xlabel('time (sec)');
% subplot(1,2,2)
% plot(tsim, F_list(2,:), 'r-', 'LineWidth', 2);
% ylabel('Right constraint (N)');
% xlabel('time (sec)');
% hold off
% 
% figure
% plot(tsim, xsim(:,4), 'LineWidth', 2);
% ylabel('Joint angle');
% xlabel('time (sec)');
% hold off

% plot joint angles
% figure;
% plot(tsim,xsim(:,3:5),'LineWidth',2);  
% ylabel('Joint Angles')
% xlabel('time (sec)')
% legend({'boardTheta','bottomLinkTheta','topLinkTheta'});


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

%% sensor.m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Description:
%   Computes the sensor values that are sent to the digital controller 
%
% Inputs:
%   x_read: the 12x1 state vector at the time of a read (note that not all
%   of the state is actually read ... we must determine the sensor readings
%   from x_read)
%   u: the control inputs (used here because we have to call
%   robot_dynamics)
%
% Outputs:
%   y: the sensor values

function [y] = sensor(x_read)
    
    % NOTE:  right now, sensors are "perfect" -- no noise or quantization.
    % That *should* be added!
    y = zeros(3,1);
    % assume encoders for bottom link angle and top link angle, could multiply x_read by something to simulate noise
    y(1:2) = x_read(4:5);   % bottomLinkTheta and topLinkTheta
    % assume IMU gyro for board angular velocity
    y(3) = x_read(8);
    
end
%% end of sensor.m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% [FLAT CASE] digital_controller.m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Description:
%   Computes the torque commands to the two motors 
%
% Inputs:
%   y: the 5x1 output vector at the time of a read 
%   memory: a struct that holds stored variables
%
% Outputs:
%   u: the two torque commands
%   memory: a struct that holds stored variables

function [u,memory] = digital_controller(y,Gains,memory)
    
    % estimate bottomLinkTheta and topLinkTheta by backwards difference
    % differentiation
    % NOTE: some low pass filtering should probably be added
    bottomLinkThetaDot = (y(1) - memory.y(1))/params.sim.dt;
    topLinkThetaDot = (y(2) - memory.y(2))/params.sim.dt;
    
    % estimate boardTheta from IMU readings
    % two approaches:  1) integrate boardThetaDot;  2) use the measured
    % accelerations and the knowledge of track shape to estimate it.  This
    % approach depends on a model, but doesn't have the problem of drift
    %
    % Approach 1:  Integrate gyro
    boardTheta = memory.boardTheta + y(3)*params.sim.dt;
    
    % package up the state estimate
    x = [0;0;boardTheta;y(1);y(2);0;0;y(3);bottomLinkThetaDot;topLinkThetaDot];
    
    % compute the controls
    error = x - memory.x_eq;
    error_3DOF = [error(3);error(4);error(5);error(8);error(9);error(10)];
    u = memory.u_eq - Gains*error_3DOF;
    
    % I've removed saturation to make it work.  If I put this back, things
    % go haywire after a while.  Probably by increasing the gain a bit, we
    % could bring this back.  For now, it might be better just to plot
    % actuator torques
    
    % Saturate u (i.e., observe actuator limitations!)
%     if abs(u(1)) > params.motor.spine.peaktorque
%         u(1) = params.motor.spine.peaktorque*sign(u(1));
%     end
%     if abs(u(2)) > params.motor.body.peaktorque
%         u(2) = params.motor.body.peaktorque*sign(u(2));
%     end
    
    % Update memory (these are values that the Tiva would store)
    memory.y = y;
    memory.boardTheta = boardTheta;
    
end
%% end of digital_controller.m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

% function [dx] = robot_dynamics(~,x,u)
% 
%     dx = zeros(numel(x),1);
%     nq = 5;    
%     % for convenience, define q_dot 
%     q_dot = x(nq+1:2*nq);
% 
%     % set up generalized forces based on motor torques
%     Q = [0;0;0;u];
%     
%     % compute M and H
%     H = H_eom(x,params);
%     M = mass_matrix(x,params); % NOTE: eliminated use of symbolic inverse mass matrix since it takes so long to compute
% 
%     dx(1:nq) = q_dot; 
%     dx(nq+1:2*nq) = M\(Q - H);
% 
% end
% 
function [dx, F] = robot_dynamics(t,x,u)
% function [dx, F] = robot_dynamics(t,x)


% for convenience, define q_dot

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

% % inputs for feedback control: board Current Angle, bottomLink Current Angle,
% % bottomLink Desired Angle, topLink Current Angle, topLink Desired Angle
% 
% [bottomMotorTorque, topMotorTorque] = pid_angle(x(3), x(4), 1.1, x(5), 0);
%  
% 
% Q = [0; 0; 0; bottomMotorTorque; topMotorTorque];
Q = [0;0;0;u];


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

%% end of main.m
end






