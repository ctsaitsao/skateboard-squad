function B = autogen_upright_input_matrix(boardI,boardMass,bottomLinkI,bottomLinkRCoM,bottomLinkMass,bottomLinkHeight,topLinkI,topLinkRCoM,topLinkMass)
%AUTOGEN_UPRIGHT_INPUT_MATRIX
%    B = AUTOGEN_UPRIGHT_INPUT_MATRIX(BOARDI,BOARDMASS,BOTTOMLINKI,BOTTOMLINKRCOM,BOTTOMLINKMASS,BOTTOMLINKHEIGHT,TOPLINKI,TOPLINKRCOM,TOPLINKMASS)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    01-May-2020 09:54:07

t2 = boardMass.*bottomLinkHeight;
t3 = bottomLinkRCoM.*bottomLinkMass;
t4 = bottomLinkMass.*bottomLinkHeight;
t5 = boardMass.*topLinkRCoM;
t6 = bottomLinkMass.*topLinkRCoM;
t7 = bottomLinkRCoM.^2;
t8 = bottomLinkHeight.^2;
t9 = topLinkRCoM.^2;
t10 = boardI.*bottomLinkI.*topLinkI.*topLinkMass;
t12 = boardI.*boardMass.*bottomLinkI.*topLinkI;
t13 = boardI.*bottomLinkI.*bottomLinkMass.*topLinkI;
t11 = -t3;
t14 = boardMass.*bottomLinkRCoM.*t3.*topLinkI;
t15 = boardI.*t5.*topLinkRCoM.*topLinkMass;
t16 = boardI.*t6.*topLinkRCoM.*topLinkMass;
t17 = bottomLinkHeight.*t2.*topLinkI.*topLinkMass;
t18 = bottomLinkRCoM.*t3.*topLinkI.*topLinkMass;
t19 = bottomLinkHeight.*t4.*topLinkI.*topLinkMass;
t20 = bottomLinkHeight.*t3.*topLinkI.*topLinkMass.*2.0;
t21 = t5.*topLinkI.*topLinkRCoM.*topLinkMass;
t22 = t6.*topLinkI.*topLinkRCoM.*topLinkMass;
t23 = t2.*topLinkI.*topLinkRCoM.*topLinkMass.*2.0;
t24 = t3.*topLinkI.*topLinkRCoM.*topLinkMass.*2.0;
t25 = t4.*topLinkI.*topLinkRCoM.*topLinkMass.*2.0;
t50 = boardI.*bottomLinkHeight.*t3.*topLinkI.*topLinkMass.*-2.0;
t51 = bottomLinkI.*bottomLinkHeight.*t3.*topLinkI.*topLinkMass.*-2.0;
t52 = boardI.*t3.*topLinkI.*topLinkRCoM.*topLinkMass.*-2.0;
t53 = bottomLinkI.*t3.*topLinkI.*topLinkRCoM.*topLinkMass.*-2.0;
t54 = bottomLinkRCoM.*t3.*t5.*topLinkRCoM.*topLinkMass;
t26 = -t20;
t27 = -t24;
t28 = boardI.*t14;
t29 = bottomLinkI.*t14;
t30 = bottomLinkI.*t15;
t31 = boardI.*t17;
t32 = bottomLinkI.*t16;
t33 = boardI.*t18;
t34 = boardI.*t19;
t35 = bottomLinkI.*t17;
t36 = bottomLinkI.*t18;
t37 = bottomLinkI.*t19;
t38 = boardI.*t20;
t39 = bottomLinkI.*t20;
t40 = t15.*topLinkI;
t41 = t16.*topLinkI;
t42 = bottomLinkI.*t21;
t43 = bottomLinkI.*t22;
t44 = boardI.*t23;
t45 = boardI.*t24;
t46 = boardI.*t25;
t47 = bottomLinkI.*t23;
t48 = bottomLinkI.*t24;
t49 = bottomLinkI.*t25;
t55 = bottomLinkRCoM.*t3.*t15;
t56 = bottomLinkI.*t54;
t57 = t2+t4+t5+t6+t11;
t58 = t10+t12+t13+t28+t29+t30+t31+t32+t33+t34+t35+t36+t37+t40+t41+t42+t43+t44+t46+t47+t49+t50+t51+t52+t53+t55+t56;
t59 = 1.0./t58;
t60 = boardI.*t57.*t59.*topLinkRCoM.*topLinkMass;
t61 = -t60;
B = reshape([0.0,0.0,0.0,0.0,0.0,boardI.*t59.*(t3.*topLinkI+bottomLinkHeight.*topLinkI.*topLinkMass+t3.*t9.*topLinkMass+topLinkI.*topLinkRCoM.*topLinkMass),0.0,-t59.*(t14+t17+t18+t19+t21+t22+t23+t25+t26+t27+t54),t59.*(t14+t15+t16+t17+t18+t19+t21+t22+t23+t25+t26+t27+t54+boardI.*boardMass.*topLinkI+boardI.*bottomLinkMass.*topLinkI+boardI.*topLinkI.*topLinkMass),t61,0.0,0.0,0.0,0.0,0.0,-t59.*topLinkRCoM.*topLinkMass.*(-boardI.*bottomLinkI+boardI.*bottomLinkRCoM.*t11+boardI.*bottomLinkHeight.*t3+bottomLinkI.*bottomLinkRCoM.*t11+bottomLinkI.*bottomLinkHeight.*t3+boardI.*t3.*topLinkRCoM+bottomLinkI.*t3.*topLinkRCoM),0.0,-bottomLinkI.*t57.*t59.*topLinkRCoM.*topLinkMass,t61,t59.*(t15+t16+boardI.*boardMass.*bottomLinkI+boardI.*bottomLinkI.*bottomLinkMass+boardI.*bottomLinkI.*topLinkMass+boardI.*bottomLinkRCoM.*t3.*topLinkMass+boardI.*bottomLinkHeight.*t2.*topLinkMass-boardI.*bottomLinkHeight.*t3.*topLinkMass.*2.0+boardI.*bottomLinkHeight.*t4.*topLinkMass+bottomLinkI.*bottomLinkRCoM.*t3.*topLinkMass+bottomLinkI.*bottomLinkHeight.*t2.*topLinkMass-bottomLinkI.*bottomLinkHeight.*t3.*topLinkMass.*2.0+bottomLinkI.*bottomLinkHeight.*t4.*topLinkMass+boardI.*t2.*topLinkRCoM.*topLinkMass.*2.0-boardI.*t3.*topLinkRCoM.*topLinkMass.*2.0+boardI.*t4.*topLinkRCoM.*topLinkMass.*2.0+bottomLinkI.*t2.*topLinkRCoM.*topLinkMass.*2.0-bottomLinkI.*t3.*topLinkRCoM.*topLinkMass.*2.0+bottomLinkI.*t4.*topLinkRCoM.*topLinkMass.*2.0+bottomLinkI.*t5.*topLinkRCoM.*topLinkMass+bottomLinkI.*t6.*topLinkRCoM.*topLinkMass+boardI.*boardMass.*bottomLinkRCoM.*t3+boardMass.*bottomLinkI.*bottomLinkRCoM.*t3)],[10,2]);
