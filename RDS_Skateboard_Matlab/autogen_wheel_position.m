function x_wheel = autogen_wheel_position(boardX,boardTheta,boardHeight,boardLength)
%AUTOGEN_WHEEL_POSITION
%    X_WHEEL = AUTOGEN_WHEEL_POSITION(BOARDX,BOARDTHETA,BOARDHEIGHT,BOARDLENGTH)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    31-May-2020 14:22:08

x_wheel = boardX-(boardLength.*cos(boardTheta))./2.0+(boardHeight.*sin(boardTheta))./2.0;
