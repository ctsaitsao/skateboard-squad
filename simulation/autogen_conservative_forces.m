function G_q = autogen_conservative_forces(boardMass,boardTheta,boardHeight,bottomLinkRCoM,bottomLinkMass,bottomLinkTheta,bottomLinkHeight,g,topLinkRCoM,topLinkMass,topLinkTheta)
%AUTOGEN_CONSERVATIVE_FORCES
%    G_Q = AUTOGEN_CONSERVATIVE_FORCES(BOARDMASS,BOARDTHETA,BOARDHEIGHT,BOTTOMLINKRCOM,BOTTOMLINKMASS,BOTTOMLINKTHETA,BOTTOMLINKHEIGHT,G,TOPLINKRCOM,TOPLINKMASS,TOPLINKTHETA)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    30-Dec-2020 13:20:46

t2 = sin(boardTheta);
t3 = boardTheta+bottomLinkTheta;
t4 = sin(t3);
t5 = t3+topLinkTheta;
t8 = (boardHeight.*t2)./2.0;
t6 = sin(t5);
t7 = bottomLinkHeight.*t4;
t9 = t6.*topLinkRCoM;
G_q = [0.0,g.*(boardMass+bottomLinkMass+topLinkMass),-g.*(topLinkMass.*(t7+t8+t9)+bottomLinkMass.*(t8+bottomLinkRCoM.*t4)),-g.*(topLinkMass.*(t7+t9)+bottomLinkRCoM.*bottomLinkMass.*t4),-g.*t9.*topLinkMass];
