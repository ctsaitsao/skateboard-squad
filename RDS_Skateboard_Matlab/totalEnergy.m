

function TE = totalEnergy(x,params)



%boardI,boardY,boardDX,boardDY,
% boardMass,boardTheta,boardDTheta,bottomLinkI,
% bottomLinkRCoM,bottomLinkMass,bottomLinkTheta,
% bottomLinkDTheta,bottomLinkHeight,g,topLinkI,topLinkRCoM,
% topLinkMass,topLinkTheta,topLinkDTheta

TE = autogen_total_energy(params.boardI,...
                  x(2),...
                  x(6),...
                  x(7),...
                  params.boardMass,...
                  x(4),...
                  x(8),...
                  params.bottomLinkI,...
                  params.bottomLinkYCoM,...
                  params.bottomLinkMass,...
                  x(4),...
                  x(9),...
                  params.bottomLinkHeight,...
                  params.g,...
                  params.topLinkI,...
                  params.topLinkYCoM,...
                  params.topLinkMass,...
                  x(5),...
                  x(10));
end