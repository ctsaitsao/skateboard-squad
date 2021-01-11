function [A_all_flat,H_constL_flat,H_constR_flat] = autogen_constraint_derivatives_flat(boardTheta,boardHeight,boardLength,wheelRadius)
%AUTOGEN_CONSTRAINT_DERIVATIVES_FLAT
%    [A_ALL_FLAT,H_CONSTL_FLAT,H_CONSTR_FLAT] = AUTOGEN_CONSTRAINT_DERIVATIVES_FLAT(BOARDTHETA,BOARDHEIGHT,BOARDLENGTH,WHEELRADIUS)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    30-Dec-2020 13:19:12

t2 = cos(boardTheta);
t3 = sin(boardTheta);
t4 = t2.*wheelRadius;
t5 = t3.*wheelRadius;
t8 = (boardHeight.*t2)./2.0;
t9 = (boardLength.*t2)./2.0;
t10 = (boardHeight.*t3)./2.0;
t11 = (boardLength.*t3)./2.0;
t6 = -t4;
t7 = -t5;
t12 = -t8;
t13 = -t10;
A_all_flat = reshape([0.0,0.0,-1.0,-1.0,t7+t9+t13,t7-t9+t13,0.0,0.0,0.0,0.0],[2,5]);
if nargout > 1
    H_constL_flat = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6-t11+t12,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[5,5]);
end
if nargout > 2
    H_constR_flat = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6+t11+t12,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[5,5]);
end
