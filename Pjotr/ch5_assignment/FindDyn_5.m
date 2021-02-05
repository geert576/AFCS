%================================================
%     Matlab Script File used to linearize the 
%     non-linear F-16 model. The program will 
%     Extract the longitudal and lateral 
%     direction matrices.  These system matrices 
%     will be used to create pole-zero mapping
%     and the bode plots of each to each control
%     input.
% Author: Richard S. Russell
% 
%================================================
clear;

addpath obsmutoolsfornewermatlabversions -END % required for some new MATLAB versions

global fi_flag_Simulink

newline = sprintf('\n');

%% Trim aircraft to desired altitude and velocity
%%
%altitude = input('Enter the altitude for the simulation (ft)  :  ');
%velocity = input('Enter the velocity for the simulation (ft/s):  ');

altitude = 20000; % [ft]
velocity = 600; % [ft/s]

xa = 15*0.3048; % distance accelerometer forward of c.g. [m]
gd = 9.80665; % gravity constant [m/s^2]
%% Initial guess for trim
%%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

%% Find trim for Hifi model at desired altitude and velocity
%%
disp('Trimming High Fidelity Model:');
fi_flag_Simulink = 1;
[trim_state_hi, trim_thrust_hi, trim_control_hi, dLEF, xu_hi] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

%% Find the state space model for the hifi model at the desired alt and vel.
%%
trim_state_lin = trim_state_hi; trim_thrust_lin = trim_thrust_hi; trim_control_lin = trim_control_hi;
[A_hi,B_hi,C_hi,D_hi] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3); ...
		dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);

%% Find trim for Hifi model at desired altitude and velocity
%%
disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

%% Find the state space model for the hifi model at the desired alt and vel.
%%
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
[A_lo,B_lo,C_lo,D_lo] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3);...
		dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);

%% Make state space model
%%
SS_hi = ss(A_hi,B_hi,C_hi,D_hi);
SS_lo = ss(A_lo,B_lo,C_lo,D_lo);


%% Make MATLAB matrix
%%
mat_hi = [A_hi B_hi; C_hi D_hi];
mat_lo = [A_lo B_lo; C_lo D_lo];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Longitudal Directional %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Select the components that make up the longitude A matrix
%%
A_longitude_hi = mat_hi([3 5 7 8 11 13 14], [3 5 7 8 11 13 14]);
A_longitude_lo = mat_lo([3 5 7 8 11 13 14], [3 5 7 8 11 13 14]);

%% Select the components that make up the longitude B matrix
%%
B_longitude_hi = mat_hi([3 5 7 8 11 13 14], [19 20]);
B_longitude_lo = mat_lo([3 5 7 8 11 13 14], [19 20]);

%% Select the components that make up the longitude C matrix
%%
C_longitude_hi = mat_hi([21 23 25 26 29], [3 5 7 8 11 13 14]);
C_longitude_lo = mat_lo([21 23 25 26 29], [3 5 7 8 11 13 14]);

%% Select the components that make up the longitude D matrix
%%
D_longitude_hi = mat_hi([21 23 25 26 29], [19 20]);
D_longitude_lo = mat_lo([21 23 25 26 29], [19 20]);

SS_long_hi = ss(A_longitude_hi, B_longitude_hi, C_longitude_hi, D_longitude_hi);
SS_long_lo = ss(A_longitude_lo, B_longitude_lo, C_longitude_lo, D_longitude_lo);

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lateral Directional %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Select the components that make up the lateral A matrix
%%
A_lateral_hi = mat_hi([4 6 7 9 10 12 13 15 16], [4 6 7 9 10 12 13 15 16]);
A_lateral_lo = mat_lo([4 6 7 9 10 12 13 15 16], [4 6 7 9 10 12 13 15 16]);

%% Select the components that make up the lateral B matrix
%%
B_lateral_hi = mat_hi([4 6 7 9 10 12 13 15 16], [19 21 22]);
B_lateral_lo = mat_lo([4 6 7 9 10 12 13 15 16], [19 21 22]);

%% Select the components that make up the lateral C matrix
%%
C_lateral_hi = mat_hi([22 24 25 27 28 30], [4 6 7 9 10 12 13 15 16]);
C_lateral_lo = mat_lo([22 24 25 27 28 30], [4 6 7 9 10 12 13 15 16]);

%% Select the components that make up the lateral D matrix
%%
D_lateral_hi = mat_hi([22 24 25 27 28 30], [19 21 22]);
D_lateral_lo = mat_lo([22 24 25 27 28 30], [19 21 22]);

SS_lat_hi = ss(A_lateral_hi, B_lateral_hi, C_lateral_hi, D_lateral_hi);
SS_lat_lo = ss(A_lateral_lo, B_lateral_lo, C_lateral_lo, D_lateral_lo);

%% Make longitudal direction SYSTEM matrix
%%
sys_long_hi = pck(A_longitude_hi, B_longitude_hi, C_longitude_hi, D_longitude_hi);
sys_long_lo = pck(A_longitude_lo, B_longitude_lo, C_longitude_lo, D_longitude_lo);

%% Make lateral direction SYSTEM matrix and Find poles for hifi
%%
sys_lat_hi = pck(A_lateral_hi, B_lateral_hi, C_lateral_hi, D_lateral_hi);

long_poles_hi = spoles(sys_long_hi);
lat_poles_hi = spoles(sys_lat_hi);



%% Make lateral direction SYSTEM matrix and Find poles for lofi
%%
sys_lat_lo = pck(A_lateral_lo, B_lateral_lo, C_lateral_lo, D_lateral_lo);

long_poles_lo = spoles(sys_long_lo);
lat_poles_lo = spoles(sys_lat_lo);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Create longitudinal SS system
A_long = SS_long_lo.A([3 4 2 5],[3 4 2 5]);
B_long = SS_long_lo.A([3 4 2 5], 7); % leave out the thrust setting
C_long = SS_long_lo.C([3 4 2 5],[3 4 2 5]); % output all states
D_long = SS_long_lo.D([3 4 2 5], 2); % 4x1 vector of zeros
SS_long_std = ss(A_long, B_long, C_long, D_long);


%% Create lateral SS system

A_lat = SS_lat_lo.A([4 1 5 6],[4 1 5 6]);
B_lat = SS_lat_lo.A([4 1 5 6],[8 9]);
C_lat = SS_lat_lo.C([4 1 5 6],[4 1 5 6]);
D_lat = SS_lat_lo.D([4 1 5 6],[2 3]);
SS_lat_std = ss(A_lat, B_lat, C_lat, D_lat);

%% Longitudinal open-loop analysis
lambda_longitudinal = eig(A_long);
lambda_sp = lambda_longitudinal(1);
w0_sp = abs(lambda_sp);
zeta_sp = -real(lambda_sp)/w0_sp;
wn_sp = w0_sp*sqrt(1-zeta_sp^2);

lambda_phug = lambda_longitudinal(3);
w0_phug = abs(lambda_phug);
zeta_phug = -real(lambda_phug)/w0_phug;
wn_phug = w0_phug*sqrt(1-zeta_phug^2);

%% simulate phugoid/short_period response
p1 = 10; %sec, breakpoint 1
p2 = 30; %sec, breakpoint 2
p3 = 700; %sec, endtime
dt = 0.001;

t = 0:dt:p3; %sec, time vector]
u1 = zeros(1,p1/dt);
u2 = 0.2*ones(1,p2/dt);
u3 = zeros(1,(p3-p2-p1+dt)/dt);

u = cat(2,u1,u2,u3); % concatenate input signal
trim_cond = [600 3.4044 3.4044 0]; % V, alpha, theta, pitchrate

phug_data = lsim(SS_long_std,u,t); % zero initial conditions
axis_fontsize = 18; %like x-axis numbers
legend_fontsize = 22;
lwdth = 2;
lblsize = 22; % [deg] indicator 
figure(1)

subplot(5,1,1)

plot(t,u,'g','LineWidth',lwdth)
set(gca,'FontSize',axis_fontsize)
grid on
legend('$\delta_e$','Interpreter','latex','FontSize',legend_fontsize)

ylabel('[deg]','FontSize',lblsize)

subplot(5,1,2)
plot(t,phug_data(:,1)+trim_cond(1),'c','LineWidth',lwdth)
set(gca,'FontSize',axis_fontsize)
grid on
legend('$V$','Interpreter','latex','FontSize',legend_fontsize)
ylabel('[ft/s]','FontSize',lblsize)

subplot(5,1,3)
plot(t,phug_data(:,2)+trim_cond(2),'m','LineWidth',lwdth)
set(gca,'FontSize',axis_fontsize)
grid on
legend('$\alpha$','Interpreter','latex','FontSize',legend_fontsize)
ylabel('[deg]','FontSize',lblsize)


subplot(5,1,4)
plot(t,phug_data(:,3)+trim_cond(3),'m','LineWidth',lwdth)
set(gca,'FontSize',axis_fontsize)
grid on
legend('$\theta$','Interpreter','latex','FontSize',legend_fontsize)
ylabel('[deg]','FontSize',lblsize)


subplot(5,1,5)
plot(t,phug_data(:,4)+trim_cond(4),'LineWidth',lwdth)
set(gca,'FontSize',axis_fontsize)
grid on
legend('$q$','Interpreter','latex','FontSize',legend_fontsize)
ylabel('[deg/s]','FontSize',lblsize)
xlabel('Time [s]','FontSize',lblsize)

sgtitle('Phugoid')

%% Lateral open-loop analysis
lambda_lateral = eig(A_lat); %extract the eigenvalues

lambda_droll = lambda_lateral(1);
w0_droll = abs(lambda_droll);
zeta_droll = -real(lambda_droll)/w0_droll;
wn_droll = w0_droll*sqrt(1-zeta_droll^2);
P_droll = 2*pi/wn_droll;
T12_droll = log(0.5)/real(lambda_droll);
Tau_droll = -1/real(lambda_droll);

lambda_aproll = lambda_lateral(3);
P_aproll = 2*pi/real(-lambda_aproll);
T12_aproll = log(0.5)/real(lambda_aproll);
Tau_aproll = -1/real(lambda_aproll)

lambda_spiral = lambda_lateral(4);
P_spiral = 2*pi/real(-lambda_spiral);
T12_spiral = log(0.5)/real(lambda_spiral);
Tau_spiral = -1/real(lambda_spiral)

%% simulate Dutch roll / aperiodic roll / spiral

a = 2; %1 = dutch roll, 2 = aperiodic roll, 3 = spiral
switch a
    case 1
        p1 = 1; %sec, time pulse starts
        p2 = 0.5; %sec, time it is doing the pulse
        p3 = 15; %sec, endtime
        dt = 0.001;
        
        t = 0:dt:p3; %sec, time vector]
        u1 = zeros(1,p1/dt);
        u2 = 1.5*ones(1,p2/dt);
        u4 = -1.5*ones(1,p2/dt); %for creating the doublet, works better for dutch roll
        u3 = zeros(1,(p3-2*p2-p1+dt)/dt);
        u = cat(2,u1,u2,u4,u3); % concatenate input signal
        u_zero = zeros(1,length(u));
        u = cat(1,u_zero,u);
        
    case 2
        p1 = 1; %sec, time pulse starts
        p2 = 1; %sec, time it is doing the pulse
        p3 = 20; %sec, endtime
        dt = 0.001;
        t = 0:dt:p3; %sec, time vector]
        u1 = zeros(1,p1/dt);
        u2 = 1.5*ones(1,p2/dt);
        u3 = zeros(1,(p3-p2-p1+dt)/dt);
        u = cat(2,u1,u2,u3); % concatenate input signal
        u_zero = zeros(1,length(u));
        u = cat(1,u,u_zero);   
        
    case 3
        p1 = 1; %sec, time pulse starts
        p2 = 1; %sec, time it is doing the pulse
        p3 = 200; %sec, endtime
        dt = 0.001;
        t = 0:dt:p3; %sec, time vector]
        u1 = zeros(1,p1/dt);
        u2 = 1.5*ones(1,p2/dt);
        u3 = zeros(1,(p3-p2-p1+dt)/dt);
        u = cat(2,u1,u2,u3); % concatenate input signal
        u_zero = zeros(1,length(u));
        u = cat(1,u,u_zero);
end

trim_cond = [0 0 0 0]; % beta, phi, p, r (all zero obviously, wings-level steady flight..

asym_data = lsim(SS_lat_std,u,t); % zero initial conditions
axis_fontsize = 18; %like x-axis numbers
legend_fontsize = 22;
lwdth = 2;
lblsize = 22; % [deg] indicator 
figure(2)

subplot(6,1,1)
plot(t,u(1,:),'g','LineWidth',lwdth)
set(gca,'FontSize',axis_fontsize)
grid on
legend('$\delta_a$','Interpreter','latex','FontSize',legend_fontsize)
ylabel('[deg]','FontSize',lblsize)

subplot(6,1,2)
plot(t,u(2,:),'g','LineWidth',lwdth)
set(gca,'FontSize',axis_fontsize)
grid on
legend('$\delta_r$','Interpreter','latex','FontSize',legend_fontsize)
ylabel('[deg]','FontSize',lblsize)

subplot(6,1,3)
plot(t,asym_data(:,1)+trim_cond(1),'m','LineWidth',lwdth)
set(gca,'FontSize',axis_fontsize)
grid on
legend('$\beta$','Interpreter','latex','FontSize',legend_fontsize)
ylabel('[deg]','FontSize',lblsize)

subplot(6,1,4)
plot(t,asym_data(:,2)+trim_cond(2),'m','LineWidth',lwdth)
set(gca,'FontSize',axis_fontsize)
grid on
legend('$\phi$','Interpreter','latex','FontSize',legend_fontsize)
ylabel('[deg]','FontSize',lblsize)


subplot(6,1,5)
plot(t,asym_data(:,3)+trim_cond(3),'LineWidth',lwdth)
set(gca,'FontSize',axis_fontsize)
grid on
legend('$p$','Interpreter','latex','FontSize',legend_fontsize)
ylabel('[deg/s]','FontSize',lblsize)


subplot(6,1,6)
plot(t,asym_data(:,4)+trim_cond(4),'LineWidth',lwdth)
set(gca,'FontSize',axis_fontsize)
grid on
legend('$r$','Interpreter','latex','FontSize',legend_fontsize)
ylabel('[deg/s]','FontSize',lblsize)
xlabel('Time [s]','FontSize',lblsize)

sgtitle('Aperiodic Roll')
