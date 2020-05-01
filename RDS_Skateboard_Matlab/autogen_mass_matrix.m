function M = autogen_mass_matrix(boardI,boardMass,boardTheta,bottomLinkI,bottomLinkRCoM,bottomLinkMass,bottomLinkTheta,bottomLinkHeight,topLinkI,topLinkRCoM,topLinkMass,topLinkTheta)
%AUTOGEN_MASS_MATRIX
%    M = AUTOGEN_MASS_MATRIX(BOARDI,BOARDMASS,BOARDTHETA,BOTTOMLINKI,BOTTOMLINKRCOM,BOTTOMLINKMASS,BOTTOMLINKTHETA,BOTTOMLINKHEIGHT,TOPLINKI,TOPLINKRCOM,TOPLINKMASS,TOPLINKTHETA)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    30-Apr-2020 22:22:13

t2 = cos(topLinkTheta);
t3 = boardTheta+bottomLinkTheta;
t4 = boardTheta.*2.0;
t5 = bottomLinkTheta.*2.0;
t6 = bottomLinkRCoM.^2;
t7 = bottomLinkHeight.^2;
t8 = topLinkTheta.*2.0;
t9 = topLinkRCoM.^2;
t10 = cos(t3);
t11 = t3+topLinkTheta;
t12 = bottomLinkHeight.*t2;
t17 = t3.*2.0;
t21 = (bottomLinkMass.*t6)./2.0;
t22 = (t7.*topLinkMass)./2.0;
t23 = (t9.*topLinkMass)./2.0;
t13 = cos(t11);
t14 = t12.*topLinkRCoM.*topLinkMass;
t15 = bottomLinkRCoM.*bottomLinkMass.*t10;
t16 = bottomLinkHeight.*t10.*topLinkMass;
t19 = cos(t17);
t20 = t3+t11;
t28 = t8+t17;
t18 = t13.*topLinkRCoM.*topLinkMass;
t24 = -t15;
t25 = cos(t20);
t26 = -t16;
t29 = cos(t28);
t33 = t19.*t21;
t34 = t19.*t22;
t27 = -t18;
t30 = bottomLinkHeight.*t25;
t32 = t29.*topLinkRCoM;
t35 = t23.*t29;
t31 = t30.*topLinkRCoM.*topLinkMass;
t36 = t24+t26+t27;
t37 = t12+t30+t32+topLinkRCoM;
t38 = (t37.*topLinkRCoM.*topLinkMass)./2.0;
t39 = t14+t21+t22+t23+t31+t33+t34+t35;
M = reshape([boardMass+bottomLinkMass+topLinkMass,0.0,t36,t36,t27,0.0,2.0,0.0,0.0,0.0,t36,0.0,boardI+t39,t39,t38,t36,0.0,t39,bottomLinkI+t39,t38,t27,0.0,t38,t38,t23+t35+topLinkI],[5,5]);
