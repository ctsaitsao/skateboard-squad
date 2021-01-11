function constraints_ramp = autogen_constraints_ramp(boardX,boardY,boardTheta,boardHeight,boardLength,normLeftX,normLeftY,normRightX,normRightY,pLeftX,pLeftY,pRightX,pRightY,wheelRadius)
%AUTOGEN_CONSTRAINTS_RAMP
%    CONSTRAINTS_RAMP = AUTOGEN_CONSTRAINTS_RAMP(BOARDX,BOARDY,BOARDTHETA,BOARDHEIGHT,BOARDLENGTH,NORMLEFTX,NORMLEFTY,NORMRIGHTX,NORMRIGHTY,PLEFTX,PLEFTY,PRIGHTX,PRIGHTY,WHEELRADIUS)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    30-Dec-2020 13:19:10

t2 = cos(boardTheta);
t3 = sin(boardTheta);
t6 = boardHeight./2.0;
t4 = t2.*wheelRadius;
t5 = t3.*wheelRadius;
t7 = t2.*t6;
t8 = (boardLength.*t2)./2.0;
t9 = t3.*t6;
t10 = (boardLength.*t3)./2.0;
constraints_ramp = [normLeftX.*(boardX-pLeftX+t5-t8+t9)-normLeftY.*(-boardY+pLeftY+t4-t6+t7+t10-wheelRadius);normRightX.*(boardX-pRightX+t5+t8+t9)+normRightY.*(boardY-pRightY-t4+t6+t10+wheelRadius-(boardHeight.*t2)./2.0)];
