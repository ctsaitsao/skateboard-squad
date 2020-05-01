function FK = autogen_fwd_kin(boardX,boardY,boardMass,boardTheta,bottomLinkRCoM,bottomLinkMass,bottomLinkTheta,bottomLinkHeight,topLinkRCoM,topLinkMass,topLinkTheta,topLinkHeight)
%AUTOGEN_FWD_KIN
%    FK = AUTOGEN_FWD_KIN(BOARDX,BOARDY,BOARDMASS,BOARDTHETA,BOTTOMLINKRCOM,BOTTOMLINKMASS,BOTTOMLINKTHETA,BOTTOMLINKHEIGHT,TOPLINKRCOM,TOPLINKMASS,TOPLINKTHETA,TOPLINKHEIGHT)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    30-Apr-2020 22:21:14

t2 = boardTheta+bottomLinkTheta;
t5 = boardMass+bottomLinkMass+topLinkMass;
t3 = cos(t2);
t4 = sin(t2);
t6 = t2+topLinkTheta;
t15 = 1.0./t5;
t7 = sin(t6);
t8 = cos(t6);
t9 = bottomLinkRCoM.*t3;
t10 = bottomLinkHeight.*t3;
t11 = bottomLinkRCoM.*t4;
t12 = bottomLinkHeight.*t4;
t13 = t8.*topLinkRCoM;
t14 = t7.*topLinkRCoM;
t16 = boardY+t9;
t17 = -t11;
t18 = -t12;
t19 = boardX+t17;
t20 = boardY+t10+t13;
FK = reshape([boardX,boardY,t19,t16,boardX-t14+t18,t20,t15.*(boardX.*boardMass+bottomLinkMass.*t19-topLinkMass.*(-boardX+t12+t14)),t15.*(boardY.*boardMass+bottomLinkMass.*t16+t20.*topLinkMass),boardX+t18-t7.*topLinkHeight,boardY+t10+t8.*topLinkHeight],[2,5]);
