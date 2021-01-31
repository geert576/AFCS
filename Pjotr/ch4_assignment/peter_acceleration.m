clc; clear;

%% load state space systems generated in FindF16Dynamics.m
load('0ft.mat');
load('5ft.mat');
load('5_9ft.mat');
load('6ft.mat');
load('7ft.mat');
load('15ft.mat');

%% create simulation time and input vector
time_vector =               0:0.001:0.2;
input_signal =              ones(1,length(time_vector));

%% simulate the six different systems
y1 = lsim(zero_ft,          input_signal,time_vector);
y2 = lsim(five_ft,          input_signal,time_vector);
y3 = lsim(fivepointnine_ft, input_signal,time_vector);
y4 = lsim(six_ft,           input_signal,time_vector);
y5 = lsim(seven_ft,         input_signal,time_vector);
y6 = lsim(fifteen_ft,       input_signal,time_vector);

%% plot results
plot(time_vector,y1)
hold on
plot(time_vector,y2)
plot(time_vector,y3)
plot(time_vector,y4)
plot(time_vector,y5)
plot(time_vector,y6)
legend('xa = 0ft','xa = 5ft','xa = 5.9ft','xa = 6ft', 'xa = 7ft','xa = 15ft')
xlabel('time [s]')
ylabel('na [g]')
hold off