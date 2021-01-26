function [bottomControl, topControl] = ...
          pid_angle(boardAngle, bottomAngle, desiredBottomAngle, topAngle, desiredTopAngle) 
      
global prevBottomError
global prevTopError

      
bottomError = desiredBottomAngle - bottomAngle;
topError = desiredTopAngle - topAngle;

kp_bottom = 50;
kd_bottom = 80000/3;


kp_top = 50;
kd_top = 80000/3;


bottomControl = kp_bottom * bottomError + (bottomError - prevBottomError) * kd_bottom;
topControl = kp_top * topError + (topError - prevTopError) * kd_top;

% CONTROL LOOP FOR WHEN BOARD ANGLE STARTS INCREASING TOO MUCH


% if boardAngle > 0.1
%     
%     boardError = 0.1 - boardAngle;
%     
%     
% end


prevBottomError = bottomError;
prevTopError = topError;


end

