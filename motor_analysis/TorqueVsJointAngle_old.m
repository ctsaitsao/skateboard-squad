n=0;
theta1=0;
F=[0;17.62];
n_vect = [];
torque1=[];
torque2=[];
i = 1;
while n<=pi/2 
 theta1=n;
 theta2=2*theta1;
 Jt=[-0.15*sin(theta1)-0.15*sin(theta1+theta2),0.15*cos(theta1)+0.15*cos(theta1+theta2);-0.15*sin(theta1+theta2),0.15*cos(theta1+theta2)];
 torque=Jt*F;
 n=n+0.010;
 n_vect(i) = n;
 i = i+1;
 torque1=[torque1,torque(1,1)];
 torque2=[torque2,torque(2,1)];
end
disp(max(abs(torque1)));
disp(max(abs(torque2)));
plot(n_vect, torque1);
hold on;
plot(n_vect, torque2);
xlabel("degrees (rad)");
ylabel("torque (Nm)");
legend('Joint 1 Torque','Joint 2 Torque');