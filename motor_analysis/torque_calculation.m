%here is constants we have(units:m)
g=9.81; %acceleration of gravity
Rramp=0.6096; %radius of ramp
L=0.5*2*pi*Rramp; %total length our skateboard run on ramp
f=0.07; %coefficient of friction

%here is all variables we design(units:kg,m)
Ms=0.85;         %mass of skateboard
Ma=0.76;            %mass of amplifier
Mg=1;         %mass of other mass
Mb=0;         %mass of battery
Mt=3;            %mass of top big mass
Mtotal=Ms+Ma+Mb+Mg; %mass of all things on skateboard
di=0.1;          %inital position of center of mass away from ground(before pumping)
t=0.3;           %total time to finish pumping 
l=0.15;          %length of each link

%calculate position of center of mass
position=Mt/(Mtotal+Mt); % it means  if center of mass need to be moved for 1cm, the big mass should be moved for 1/a cm

%calculate initial velocity and final velocity during pumping
vi=sqrt(2*g*Rramp-f*g*L); %velocity before pumping
vf=sqrt(2*g*Rramp+f*g*L); %velocity after pumping
ratio=vf/vi;                 %ratio of ri/rf and vf/vi
df=((ratio-1)*Rramp+di)/ratio;%final position of center of mass away from ground(after pumping)
d=(df-di)/position;                  %distance that big mass should move
b=position*2*l;
disp('max position for center of mass=');
disp(b);
if b>=0.13
    disp('it is able to do stall');
elseif disp('it is not able to do stall')
end

%here we calculation acceleration
%if you need to use triangle acceleration map, set shape=0; if you need to use
%trapezoid acceleration map, set shape=1;
shape=0;
if shape==0
    Vpeak=2*d/t; %peak velocity in acceleration. This is derivative of triangle
    a=Vpeak/(t/2);%acceleration  
    disp('This is triangle acceleration map.');
elseif shape==1
    Vpeak=2*d/(t-0.2*t+t); %This is derivative of trapezoid and we assume we need to take 10% of total time to accelerate and decelerate
    a=Vpeak/(0.1*t);%acceleration
    disp('This is trapezoid acceleration map.');
end

%here we calculate force
Facceleration=Mt*g+Mt*vf^2/Rramp+Mt*a; %force in acceleration process
Fpeak=Mt*g+Mt*vf^2/Rramp;              %force in peak velocity process
Fdeceleration=Mt*g+Mt*vf^2/Rramp-Mt*a;  %force in deceleration process

%here we calculate distance of different process
if shape==0
    Dacceleration=0.5*Vpeak*0.5*t;
    Dpeak=0;
    Ddeceleration=0.5*Vpeak*0.5*t;
elseif shape==1
    Dacceleration=0.5*Vpeak*0.1*t;
    Dpeak=Vpeak*0.8*t;
    Ddeceleration=0.5*Vpeak*0.1*t;
end

%here we calculate power
if shape==0
    Pacceleration=Facceleration*Dacceleration/(0.5*t);
    Ppeak=0;
    Pdeceleration=Fdeceleration*Ddeceleration/(0.5*t);
elseif shape==1
    Pacceleration=Facceleration*Dacceleration/(0.1*t);
    Ppeak=Fpeak*Dpeak/(0.8*t);
    Pdeceleration=Fdeceleration*Ddeceleration/(0.1*t);
end
disp('Power of Acceleration is');
disp(Pacceleration);
disp('Power of Peak is');
disp(Ppeak);
disp('Power of Deceleration is');
disp(Pdeceleration);

%here we calculate torque we need,below are from our previous matlab code
n=0.15;% it's radian and it's equal to 8.59 degree
theta1=0;
F=[0;Facceleration]; % it's the max force of whole process
n_vect = [];
torque1=[];
torque2=[];
i = 1;
while n<=1.396 % it's radian and it's equal to 80 degree
 theta1=n;
 theta2=2*theta1;
 Jt=[-l*sin(theta1)-l*sin(theta1+theta2),l*cos(theta1)+l*cos(theta1+theta2);-l*sin(theta1+theta2),l*cos(theta1+theta2)];%it's jacobian transpose
 torque=Jt*F;
 n=n+0.010;%it's step we use for calculation,it's radian
 n_vect(i) = n;
 i = i+1;
 torque1=[torque1,torque(1,1)];
 torque2=[torque2,torque(2,1)];
end
disp('max for torque 1 is');
disp(max(abs(torque1)));
disp('max for torque 2 is');
disp(max(abs(torque2)));

%here we calculate our RPM
w1=Vpeak/(2*l*cos(80/180*pi))*60/(2*pi);
w2=w1*2;
disp('max RPM for link1 is');
disp(w1);
disp('max RPM for link2 is');
disp(w2);