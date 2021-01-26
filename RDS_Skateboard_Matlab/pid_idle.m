function [bottomControl, topControl] = ...
          pid_idle(bottomAngle, topAngle) 
      
global prevBottomErrorIdle
global prevTopErrorIdle

      
bottomError = 0 - bottomAngle;
topError = 0 - topAngle;

kp_bottom = 50;
kd_bottom = 80000/3;


kp_top = 50;
kd_top = 80000/3;


bottomControl = kp_bottom * bottomError + (bottomError - prevBottomErrorIdle) * kd_bottom;
topControl = kp_top * topError + (topError - prevTopErrorIdle) * kd_top;


prevBottomErrorIdle = bottomError;
prevTopErrorIdle = topError;


end

