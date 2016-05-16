%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Test of rkydyn ode45 function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;clc;close all

% ode settings and initial conditions
odeOpt = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
h0 = 0;
v0 = 0;
z0 = 0;
m0 = 10;
del0 = degtorad(0);
deld0 = 0;
x0 = [h0, v0, z0, m0, del0, deld0]';

cnsts.tbo   = 5;        % Time of burnout
cnsts.a     = 0;        % Thrust curve coefficient
cnsts.b     = -10;      % Thrust curve coefficient
cnsts.c     = 700;      % Thrust curve coefficient
cnsts.A     = .5;        % DC motor gain
cnsts.tao   = .25;      % DC motor time constant
cnsts.xcp   = 0.007;    % c.p. of airbrake from LE
cnsts.rho   = 1.225;    % Air density
cnsts.Ab    = 0.002;    % Area of airbrake
cnsts.Ar    = 0.05;     % Cross sectional area of rocket body
cnsts.n     = 4;        % Number of airbrakes
cnsts.th    = 0;        % Rocket angle to vertical
cnsts.ue    = 3000;     % Exhaust gas velicity
cnsts.g     = 9.81;     % Gravity
cnsts.Cdb   = 0.5;      % Airbrake drag coefficient
cnsts.Cdr   = 0.5;      % Rocket drag coefficient
cnsts.delmax= pi/4;     % Max airbrake deflection
cnsts.Ib    = 3.6e-5;   % Airbrake element moment of inertia

tf = 15;
% Run with no control
u = 0;
[tout, yout] = ode45(@(t, x) rktdyn(t, x, u, cnsts),[0 tf], x0, odeOpt);
plot(tout,yout(:,1));
hold on
plot(tout,yout(:,2));

% Run with commanded 20 degree airbrake extension
ax = gca;
ax.ColorOrderIndex = 1;
u = degtorad(45);
[tout, yout] = ode45(@(t, x) rktdyn(t, x, u, cnsts),[0 tf], x0, odeOpt);
plot(tout,yout(:,1),'--');
hold on
plot(tout,yout(:,2),'--');
legend('h','v','h (brakes out)','v (brakes out)')

% Airbrake deflection with constant voltage
figure
plot(tout,radtodeg(yout(:,5)));
title('Drag element deflection vs time')
max(yout(:,1))
% Mass plot
% figure
% plot(tout,yout(:,4));
% title('Rocket Mass vs Time')
