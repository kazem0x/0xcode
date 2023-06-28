
clc;
clear all;
close all; 

% Parameters for the system
a = 1;
b = 3;
c = 1;
d = 5;
r = 0.006;
s = 4;
I = 1.8;

% Use fsolve to find the equilibrium points
guess = [0 0 0]; % Initial guess for the equilibrium points
equilibrium_points = fsolve(@(x) myEquations(x, a, b, c, d, I, r, s), guess);

% Display the equilibrium points
disp(equilibrium_points);

% Compute the Jacobian matrix
syms x y z % define symbolic variables
J = jacobian(myEquations([x y z], a, b, c, d, I, r, s), [x y z]); % compute the Jacobian
J_subs = subs(J, [x y z], equilibrium_points); % substitute the equilibrium points
disp(J_subs); % display the Jacobian matrix at the equilibrium points


% Define the range of x and y values for the phase plane
x_range = linspace(-2, 2, 20);
y_range = linspace(-2, 2, 20);

% Create a meshgrid of x and y values
[x, y] = meshgrid(x_range, y_range);

% Compute the derivatives of x and y at each point on the grid
xdot = y - a*x.^3 + b*x.^2 + I;
ydot = c - d*x.^2 - y;
zdot = r*(s*(x) - equilibrium_points(3));

% Plot the phase plane
quiver(x, y, xdot, ydot, 'r'); % plot the vector field
hold on;
streamslice(x, y, xdot, ydot, 'cmap', 'jet'); % plot streamlines
axis tight;
title('Phase Plane');
xlabel('x');
ylabel('y');
grid on;

function f = myEquations(x, a, b, c, d, I, r, s)
% System of equations: xdot = y - a*x^3 + b*x^2 + I - z;
% ydot = c - d*x^2 - y;
% zdot = r*(s*(x - xr) - z)

f = [x(2) - a*x(1)^3 + b*x(1)^2 + I - x(3);
     c - d*x(1)^2 - x(2);
     r*(s*(x(1)) - x(3))];
end
