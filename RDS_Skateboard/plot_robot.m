%% plot_robot.m
%
% Description:
%   Plots the robot in its current configuration.
%   
% Inputs:
%   q: robot configuration, q = [boardX, boardY, boardTheta, bottomLinkX, bottomLinkY];
%   params: a struct with many elements, generated by calling init_params.m
%   varargin: optional name-value pair arguments:
%       'new_fig': (default: false), if true, plot appears on new figure
%
% Outputs:
%   none
%
% Notes:
%   1) This code is written verbosely in the hope that it is clear how to
%   extend it to visualizing other more complex robots.

function plot_robot(q,params,varargin)
%% Parse input arguments
% Note: a simple robot plotting function doesn't need this, but I want to
% write extensible code, so I'm using "varargin" which requires input
% parsing. See the reference below:
%
% https://people.umass.edu/whopper/posts/better-matlab-functions-with-the-inputparser-class/

% Step 1: instantiate an inputParser:
p = inputParser;

% Step 2: create the parsing schema:
%      2a: required inputs:
addRequired(p,'robot_config', ...
    @(q) isnumeric(q));
addRequired(p,'robot_params', ...
    @(params) ~isempty(params));
%      2b: optional inputs:
addParameter(p, 'new_fig', false); % if true, plot will be on a new figure

% Step 3: parse the inputs:
parse(p, q, params, varargin{:});

% Verification: display the results of parsing:
% disp(p.Results)


%% Compute the 4 corners of the cart, clockwise from top left corner
% First compute the cart's home position (q(1) = 0):

T_board =   [cos(q(3)), -sin(q(3)), q(1);
             sin(q(3)),  cos(q(3)), q(2);
             0,          0,         1]; 

board.home.upp_left.x    = -0.5*params.boardLength;
board.home.upp_left.y    = 0.5*params.boardHeight;

board.home.upp_right.x   = 0.5*params.boardLength;
board.home.upp_right.y   = 0.5*params.boardHeight;

board.home.low_right.x   = 0.5*params.boardLength;
board.home.low_right.y   = -0.5*params.boardHeight;

board.home.low_left.x    = -0.5*params.boardLength;
board.home.low_left.y    = -0.5*params.boardHeight;

board.home.corners = horzcat([board.home.upp_left.x; board.home.upp_left.y; 1],...
                            [board.home.upp_right.x; board.home.upp_right.y; 1],...
                            [board.home.low_right.x; board.home.low_right.y; 1],...
                            [board.home.low_left.x;  board.home.low_left.y; 1]);
                        


                        

% The cart can translate horizontally by q(1) but cannot translate
% vertically or rotate, so we don't bother computing a homogeneous
% transformation matrix in SE(2) for the cart. Instead we do the math
% explicitly:

board.curr.corners = T_board*board.home.corners;

board.curr.uppMidpoint = [(board.curr.corners(1,1) + board.curr.corners(1,2))/2,...
                               (board.curr.corners(2,1) + board.curr.corners(2,2))/2];

%% Compute the 4 corners of the bottom link
% The bottom link is a rectangle whose bottom edge's center is (boardX + offset, boardY). The bottom
% link can translate horizontally and can rotate, so we first compute a 
% homogeneous transformation matrix T_pend in SE(2):

T_bottomLink = [cos(q(3) + q(4)), -sin(q(3) + q(4)), q(1);
                sin(q(3) + q(4)),  cos(q(3) + q(4)), q(2);
                 0,          0,         1];

% We first compute the 4 corners of the pendulum when the robot is in the
% "home" configuration (q(1) = q(2) = 0):

bottomLink.home.low_right.x   = 0.5* params.bottomLinkWidth;
bottomLink.home.low_right.y   = 0;


bottomLink.home.low_left.x    = -0.5* params.bottomLinkWidth;
bottomLink.home.low_left.y    = 0;


bottomLink.home.upp_right.x   = 0.5* params.bottomLinkWidth;
bottomLink.home.upp_right.y   = params.bottomLinkHeight;

bottomLink.home.upp_left.x    = -0.5* params.bottomLinkWidth;
bottomLink.home.upp_left.y    = params.bottomLinkHeight;


bottomLink.home.uppMidpoint = [(bottomLink.home.upp_right.x + bottomLink.home.upp_left.x)/2, ...
                               (bottomLink.home.upp_right.y + bottomLink.home.upp_left.y)/2];

bottomLink.home.corners = horzcat([bottomLink.home.upp_left.x; bottomLink.home.upp_left.y;   1],...
                            [bottomLink.home.upp_right.x; bottomLink.home.upp_right.y; 1],...
                            [bottomLink.home.low_right.x; bottomLink.home.low_right.y; 1],...
                            [bottomLink.home.low_left.x;  bottomLink.home.low_left.y;  1]);
                        
% Now compute the 4 corners of the pendulum after undergoing planar
% translation + rotation as described by T_pend:

bottomLink.curr.corners = T_bottomLink*bottomLink.home.corners;

bottomLink.curr.uppMidpoint = [(bottomLink.curr.corners(1,1) + bottomLink.curr.corners(1,2))/2,...
                               (bottomLink.curr.corners(2,1) + bottomLink.curr.corners(2,2))/2];



%% Compute the 4 corners of the top link
% The top link is a rectangle where the bottom edge's midpoint is the 
% midpoint of the bottom link's top edge. The top
% link can translate horizontally and can rotate, so we first compute a 
% homogeneous transformation matrix T_pend in SE(2):
% 
T_topLink = [cos(q(3) + q(4) + q(5)), -sin(q(3) + q(4) + q(5)), bottomLink.curr.uppMidpoint(1);
             sin(q(3) + q(4) + q(5)),  cos(q(3) + q(4) + q(5)), bottomLink.curr.uppMidpoint(2);
             0,          0,         1];

% We first compute the 4 corners of the pendulum when the robot is in the
% "home" configuration (q(1) = q(2) = 0):
topLink.home.upp_left.x    = -0.5*params.topLinkWidth;
topLink.home.upp_left.y    = params.topLinkHeight;


topLink.home.upp_right.x   = 0.5*params.topLinkWidth;
topLink.home.upp_right.y   = params.topLinkHeight;

topLink.home.low_left.x    = -0.5*params.topLinkWidth;
topLink.home.low_left.y    = 0;

topLink.home.low_right.x   =  0.5*params.topLinkWidth;
topLink.home.low_right.y   =  0;



topLink.home.corners = horzcat([topLink.home.upp_left.x; topLink.home.upp_left.y;   1],...
                            [topLink.home.upp_right.x; topLink.home.upp_right.y; 1],...
                            [topLink.home.low_right.x; topLink.home.low_right.y; 1],...
                            [topLink.home.low_left.x;  topLink.home.low_left.y;  1]);

% Now compute the 4 corners of the pendulum after undergoing planar
% translation + rotation as described by T_pend:

topLink.curr.corners = T_topLink*topLink.home.corners;



% %% Evaluate forward kinematics at points of interest
% FK = fwd_kin(q,params);
% 
% % (x,y) location of cart CoM:
% board.curr.com.x = FK(1,1);
% board.curr.com.y = FK(2,1);
% 
% % (x,y) location of pendulum CoM:
% bottomLink.curr.com.x = FK(1,2);
% bottomLink.curr.com.y = FK(2,2);
% 
% % (x,y) location of pendulum tip:
% bottomLink.curr.tip.x = FK(1,3);
% bottomLink.curr.tip.y = FK(2,3);
% 
% %% Display the cart, pendulum, and the pendulum's CoM
% if p.Results.new_fig
%     figure;
% end
% 
fill(board.curr.corners(1,:),board.curr.corners(2,:),params.viz.colors.board);
hold on;
fill(bottomLink.curr.corners(1,:),bottomLink.curr.corners(2,:),params.viz.colors.bottomLink);
hold on 
fill(topLink.curr.corners(1,:),topLink.curr.corners(2,:),params.viz.colors.topLink);
plot(bottomLink.curr.com.x,bottomLink.curr.com.y,'o','MarkerSize',20,...
    'MarkerFaceColor',params.viz.colors.pend_com,...
    'MarkerEdgeColor',params.viz.colors.pend_com);
hold off;

axis(params.viz.axis_lims);
daspect([1 1 1]) % no distortion

xlabel('$x$');
ylabel('$y$');

end