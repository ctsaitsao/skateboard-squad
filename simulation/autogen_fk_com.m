function [FK_com_b,FK_com_t] = autogen_fk_com(boardX,boardY,boardTheta,bottomLinkRCoM,bottomLinkTheta,bottomLinkHeight,topLinkRCoM,topLinkTheta)
%AUTOGEN_FK_COM
%    [FK_COM_B,FK_COM_T] = AUTOGEN_FK_COM(BOARDX,BOARDY,BOARDTHETA,BOTTOMLINKRCOM,BOTTOMLINKTHETA,BOTTOMLINKHEIGHT,TOPLINKRCOM,TOPLINKTHETA)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    01-May-2020 09:51:30

t2 = boardTheta+bottomLinkTheta;
t3 = cos(t2);
t4 = sin(t2);
FK_com_b = [boardX-bottomLinkRCoM.*t4;boardY+bottomLinkRCoM.*t3];
if nargout > 1
    t5 = t2+topLinkTheta;
    FK_com_t = [boardX-bottomLinkHeight.*t4-topLinkRCoM.*sin(t5);boardY+bottomLinkHeight.*t3+topLinkRCoM.*cos(t5)];
end