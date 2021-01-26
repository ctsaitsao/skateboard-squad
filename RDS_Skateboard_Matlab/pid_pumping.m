function [bottomControl, topControl] = ...
          pid_pumping(bottomLinkTheta, topLinkTheta, direction) 
      
global prevBottomErrorPumping
global prevTopErrorPumping

if direction == 0


desiredBLTheta = -0.8;
desiredTLTheta = 1.6;

elseif direction == 1
    
desiredBLTheta = 0;
desiredTLTheta = 0;

end
    

bottomError =  desiredBLTheta - bottomLinkTheta;
topError = desiredTLTheta - topLinkTheta;

kp_bottom = 50;
kd_bottom = 3000*2;


kp_top = 100;
kd_top = 10000*2;




bottomControl = kp_bottom * bottomError + (bottomError - prevBottomErrorPumping) * kd_bottom;
topControl = kp_top * topError + (topError - prevTopErrorPumping) * kd_top;


prevBottomErrorPumping = bottomError;
prevTopErrorPumping = topError;






end

