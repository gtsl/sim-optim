%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Rocket state dynamics
%
% Per Georgia Tech Student Launch "Simplified Equations of Motion" report.
% The following dynamics describe a rocket in atmospheric ascent with
% airbrake control applied through a DC motor.
%
% The state  and control vectors are
%   x =[h, v, z, m]'    u = [V]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = rktdyn(t, x, u, cnsts)

% State
v = x(2);
z = x(3);
m = x(4);
del = x(5);
deld = x(6);

% Control
delc = u(1);

% Constants
tbo = cnsts.tbo;        % Time of burnout
a = cnsts.a;            % Thrust curve coefficient
b = cnsts.b;            % Thrust curve coefficient
c = cnsts.c;            % Thrust curve coefficient
A = cnsts.A;            % DC motor gain
tao = cnsts.tao;        % DC motor time constant
xcp = cnsts.xcp;        % c.p. of airbrake from LE
rho = cnsts.rho;        % Air density
Ab = cnsts.Ab;          % Area of airbrake
Ar = cnsts.Ar;          % Cross sectional area of rocket body
n = cnsts.n;            % Number of airbrakes
th = cnsts.th;          % Rocket angle to vertical
ue = cnsts.ue;          % Exhaust gas velicity
g = cnsts.g;            % Gravity
Cdb = cnsts.Cdb;        % Airbrake drag coefficient
Cdr = cnsts.Cdr;        % Rocket drag coefficient
delmax = cnsts.delmax;  % Max airbrake deflection
Ib = cnsts.Ib;          % Airbrake element moment of inertia
q = 0.5 * rho * v^2;    % Dynamic pressure

% Thrust calculation
if t >= tbo
    T = 0;
else
    T = a * t^3 + b * t^2 + c;
end

% Airbrake deflection feedback (deldc = 0)
V = 5 * (delc - del) - 3 * deld;
if V < -2.5
    V = -2.5;
elseif V > 2.5
    V = 2.5;
end

% Equations of motion
deldd = (1 / Ib) * (A * z / tao - q * Cdb * Ab * xcp * sin(del)^2);
vd = (T - q * (Ar * Cdr + n * Ab * Cdb * sin(del))) / m ...
    - g*cos(th);
hd = v * cos(th);
md = -T / ue;
zd = -z / tao + V;

% Pack
xd = zeros(4,1);
xd(1) = hd;
xd(2) = vd;
xd(3) = zd;
xd(4) = md;
xd(5) = deld;
xd(6) = deldd;
end