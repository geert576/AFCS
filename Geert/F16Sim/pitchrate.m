% LongLofi matrix

A = [
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

% figure(1);
% plot(Tout2, y2(:,2));
% title('Without actuator dynamics')
% 
% figure(2);
% plot(Tout4, y4(:,4));
% title('With actuator dynamics')

figure(3);
[y,tOut] = step(sys_2);
plot(tOut,y(:,2));
ylabel('Pitch rate [rad/s]')
xlabel('Time [s]')
title('Without actuator dynamics')

figure(4);
[y,tOut] = step(sys_4);
plot(tOut(1:1500),y(1:1500,4));
ylabel('Pitch rate [rad/s]')
xlabel('Time [s]')
title('With actuator dynamics')
