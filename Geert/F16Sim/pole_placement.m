syms K_a K_q s x
A = [
   -0.0109   -1.7611  -32.1700   -0.8207;
   -0.0002   -0.6505    0.0000    0.9482;
         0         0         0    1.0000;
   -0.0000   -1.9092         0   -0.8893];

B = [
    0.1093;
   -0.0014;
         0;
   -0.1389];

C = [
    1.0000         0         0         0;
         0   57.2958         0         0;
         0         0   57.2958         0;
         0         0         0   57.2958];

D = [
     0;
     0;
     0;
     0];
 
% Creating the short period model 
A_sp_uncontrolled = A([2 4] ,[2 4] );
B_sp = B([2 4]);
C_sp = C([2 4], [2 4]);
D_sp = D([2 4]);

%Controller variables
I = eye(2);
K = [K_a,K_q];

sys = ss(A_sp_uncontrolled,B_sp,C_sp,D_sp);
% sys_q = sys(2);

%Determine the poles of these state space systems
speed = 600; %ft/s
V = 0.3048*speed;

wn_req = 0.03 * V;
Time_cons = 0.75 * wn_req;
damp_req = 0.5;

% Formula from: https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-30-feedback-control-systems-fall-2010/lecture-notes/MIT16_30F10_lec11.pdf
% Find K for pole locations
A_controlled = (A_sp_uncontrolled - B_sp*K);
A_controlled
charact_equation = det(s*I - A_controlled)';

% Calculate pole required for requirments, assuming second order transfer
% function
pole_required = solve(x^2 + 2*wn_req*damp_req*x + wn_req^2)


% Try the matlab way
% K = place(sys_q.A,sys_q.B,pole_required)

% Instert poles in characteristic equation, solve for k_a and k_q
charact_equation = subs(charact_equation, s, pole_required(1));
S = solve(charact_equation,K_a, K_q);
result_K_a = double(real(S.K_a)) ;
impart_K_a = double(imag(S.K_a));
result_K_q = double(real(S.K_q)) ;
impart_K_q = double(imag(S.K_q));

% Rewrite results
K = [result_K_a + impart_K_a*i,result_K_q + impart_K_q*i];
K2 = [-190.9711, -26.4884];

% Determine properties of system with controller
A_controlled = (A_sp_uncontrolled - B_sp*K2);
sys = ss(A_controlled,B_sp,C_sp,D_sp);
damp(sys)
p = [-2.7432 + 4.7514*1i, -2.7432 - 4.7514*1i];
place(A_sp_uncontrolled,B_sp,p);