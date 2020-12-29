function M = autogen_mass_matrix(boardI,boardMass,boardTheta,boardHeight,bottomLinkI,bottomLinkRCoM,bottomLinkMass,bottomLinkTheta,bottomLinkHeight,topLinkI,topLinkRCoM,topLinkMass,topLinkTheta)
%AUTOGEN_MASS_MATRIX
%    M = AUTOGEN_MASS_MATRIX(BOARDI,BOARDMASS,BOARDTHETA,BOARDHEIGHT,BOTTOMLINKI,BOTTOMLINKRCOM,BOTTOMLINKMASS,BOTTOMLINKTHETA,BOTTOMLINKHEIGHT,TOPLINKI,TOPLINKRCOM,TOPLINKMASS,TOPLINKTHETA)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    03-Jun-2020 17:58:25

t2 = cos(boardTheta);
t3 = cos(bottomLinkTheta);
t4 = sin(boardTheta);
t5 = cos(topLinkTheta);
t6 = boardTheta+bottomLinkTheta;
t7 = bottomLinkTheta+topLinkTheta;
t8 = boardHeight.^2;
t9 = bottomLinkRCoM.^2;
t10 = bottomLinkHeight.^2;
t11 = topLinkRCoM.^2;
t16 = boardMass+bottomLinkMass+topLinkMass;
t12 = boardHeight.*t2;
t13 = cos(t6);
t14 = boardHeight.*t4;
t15 = sin(t6);
t17 = t6+topLinkTheta;
t18 = cos(t7);
t20 = bottomLinkMass.*t9;
t21 = t10.*topLinkMass;
t22 = t11.*topLinkMass;
t24 = bottomLinkHeight.*t5.*topLinkRCoM.*topLinkMass;
t36 = (boardHeight.*bottomLinkRCoM.*bottomLinkMass.*t3)./2.0;
t37 = (boardHeight.*bottomLinkHeight.*t3.*topLinkMass)./2.0;
t19 = sin(t17);
t23 = cos(t17);
t25 = bottomLinkRCoM.*t13.*2.0;
t26 = bottomLinkHeight.*t13.*2.0;
t27 = bottomLinkRCoM.*t15.*2.0;
t28 = bottomLinkHeight.*t15.*2.0;
t29 = t24.*2.0;
t30 = bottomLinkRCoM.*bottomLinkMass.*t13;
t31 = bottomLinkRCoM.*bottomLinkMass.*t15;
t42 = (boardHeight.*t18.*topLinkRCoM.*topLinkMass)./2.0;
t45 = t22+t24+topLinkI;
t32 = t23.*topLinkRCoM.*topLinkMass;
t33 = t19.*topLinkRCoM.*topLinkMass;
t34 = t23.*topLinkRCoM.*2.0;
t35 = t19.*topLinkRCoM.*2.0;
t38 = -t30;
t39 = -t31;
t43 = t12+t25;
t44 = t14+t27;
t58 = t42+t45;
t67 = bottomLinkI+t20+t21+t22+t29+t36+t37+t42+topLinkI;
t40 = -t32;
t41 = -t33;
t46 = t26+t34;
t47 = t28+t35;
t48 = (bottomLinkMass.*t44)./2.0;
t49 = (bottomLinkMass.*t43)./2.0;
t50 = -t48;
t51 = -t49;
t52 = t12+t46;
t53 = t14+t47;
t54 = (t46.*topLinkMass)./2.0;
t55 = (t47.*topLinkMass)./2.0;
t56 = -t54;
t57 = -t55;
t59 = (t53.*topLinkMass)./2.0;
t60 = (t52.*topLinkMass)./2.0;
t61 = -t59;
t62 = -t60;
t63 = t38+t56;
t64 = t39+t57;
t65 = t51+t62;
t66 = t50+t61;
M = reshape([t16,0.0,t65,t63,t40,0.0,t16,t66,t64,t41,t65,t66,boardI+bottomLinkI+t20+t21+t22+t29+topLinkI+(bottomLinkMass.*t8)./4.0+(t8.*topLinkMass)./4.0+boardHeight.*bottomLinkHeight.*t3.*topLinkMass+boardHeight.*t18.*topLinkRCoM.*topLinkMass+boardHeight.*bottomLinkRCoM.*bottomLinkMass.*t3,t67,t58,t63,t64,t67,bottomLinkI+t20+t21+t22+t29+topLinkI,t45,t40,t41,t58,t45,t22+topLinkI],[5,5]);