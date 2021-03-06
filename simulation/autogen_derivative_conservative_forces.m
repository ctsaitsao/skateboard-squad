function G_jac = autogen_derivative_conservative_forces(boardTheta,boardHeight,bottomLinkRCoM,bottomLinkMass,bottomLinkTheta,bottomLinkHeight,g,topLinkRCoM,topLinkMass,topLinkTheta)
%AUTOGEN_DERIVATIVE_CONSERVATIVE_FORCES
%    G_JAC = AUTOGEN_DERIVATIVE_CONSERVATIVE_FORCES(BOARDTHETA,BOARDHEIGHT,BOTTOMLINKRCOM,BOTTOMLINKMASS,BOTTOMLINKTHETA,BOTTOMLINKHEIGHT,G,TOPLINKRCOM,TOPLINKMASS,TOPLINKTHETA)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    30-Dec-2020 13:20:47

t2 = cos(boardTheta);
t3 = boardTheta+bottomLinkTheta;
t4 = cos(t3);
t5 = t3+topLinkTheta;
t8 = (boardHeight.*t2)./2.0;
t6 = cos(t5);
t7 = bottomLinkHeight.*t4;
t9 = bottomLinkRCoM.*bottomLinkMass.*t4;
t10 = t6.*topLinkRCoM;
t11 = g.*t10.*topLinkMass;
t13 = t7+t10;
t12 = -t11;
t14 = t13.*topLinkMass;
t15 = t9+t14;
t16 = g.*t15;
t17 = -t16;
G_jac = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-g.*(topLinkMass.*(t8+t13)+bottomLinkMass.*(t8+bottomLinkRCoM.*t4)),t17,t12,0.0,0.0,t17,t17,t12,0.0,0.0,t12,t12,t12],[5,5]);
