%for this exercise, h=5000ft, V=300ft/s 
clc; clear;
%% SS extracted from FindF16Dynamics.m A,B,C,D = 5-states, A_long, B_long... = 4-states

A =    [-0.0291    2.1300  -32.1700   -2.8952    0.0001; %v, alpha, theta, q, h
   -0.0007   -0.5447    0.0000    0.9152    0.0000;
         0         0         0    1.0000         0;
    0.0000    0.3303         0   -0.8169   -0.0000;
         0 -300.0000  300.0000         0         0];
B =   [ 0.0045    0.0015; % de, dt
   0.0011   -0.0000;
         0         0;
   0.0570         0;
         0         0];
C =     [1.0000         0         0         0         0;
         0   57.2958         0         0         0;
         0         0   57.2958         0         0;
         0         0         0   57.2958         0;
         0         0         0         0    1.0000];
D = zeros(5,2);

A_long = [-0.0291    2.1300    -32.1700   -2.8952;
                  -0.0007   -0.5447    0.0000     0.9152;
                  0          0         0          1.0000;
                  0.0000     0.3303    0          -0.8169];  
B_long = [-0.0045;   -0.0011;  0;         -0.0570];
C_long = [1.0000     0         0          0;
                 0           180/pi    0          0;
                 0           0         180/pi     0;
                 0           0         0          180/pi];    
D_long = zeros(4,1);
[V0, alpha0, theta0, q0, h0, de0, dt0] = deal(300, 10.4511, 10.4511, 0, 5000, -4.1891, 2826.8165); % ft/s, deg, deg, deg/s, ft, deg, lbs
SS_long = ss(A,B,C,D);
SS_attempt = ss(A_long,B_long,C_long,D_long);

%% gain design
s = tf('s');

qs_des = (3.266*s^4 + 1.895*s^3 + 0.05718*s^2 - 6.858e-05*s)/ (s^5 + 1.391*s^4 + 0.1838*s^3 + 0.004679*s^2 - 0.007455*s + 6.936e-06);
H_servo = 20.02/(s+20.02);

H_eq = qs_des*H_servo;

%rlocus(tf(SS_attempt(4)))
rlocus(tf(SS_attempt(4))*H_servo)


