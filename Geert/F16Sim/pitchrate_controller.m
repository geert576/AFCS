% LongLofi matrix
K_a = 1;
K_q = 1;


A_uncontrolled = [
   -0.0109   -1.7611  -32.1700   -0.8207;
   -0.0002   -0.6505    0.0000    0.9482;
         0         0         0    1.0000;
   -0.0000   -1.9092         0   -0.8893]

B = [
    0.1093;
   -0.0014;
         0;
   -0.1389]

C = [
    1.0000         0         0         0;
         0   57.2958         0         0;
         0         0   57.2958         0;
         0         0         0   57.2958]

D = [
     0;
     0;
     0;
     0]

K = [0,K_a,0,K_q];
 
% Formula from: https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-30-feedback-control-systems-fall-2010/lecture-notes/MIT16_30F10_lec11.pdf
A = (A_uncontrolled - B*K);

% Creating the short period model 
A_sp = A([2 4] ,[2 4] );
B_sp = B([2 4]);
C_sp = C([2 4], [2 4]);
D_sp = D([2 4]);

% Creating a step input
N = 100;
t = 1:1:N;
u = [linspace(0,0, N*1/10), linspace(1,1, N*9/10)];

% System response with actuator dynamics
sys_4 = ss(A,B,C,D);

% System response without actuator dynamics
sys_2 = ss(A_sp,B_sp,C_sp,D_sp);

[y2, Tout2] = lsim(sys_2,u,t);
[y4, Tout4] = lsim(sys_4,u,t);

%Determine the poles of these state space systems
speed = 600; %ft/s
V = 0.3048*speed;

wn_req = 0.75 * V;
Time_cons = 0.75 * wn_req;
damp_req = 0.5;

[wn,zeta,p] = damp(sys_2(2))

step(sys_2(2));
a = tf(sys_2(2));

rlocus(sys_2(2))