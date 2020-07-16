function x2 = rungekutta(x1, v0, v1, v2, dt)
%%%%%%%%%%%%%%%%%%%%%%%%%
% Use runge-kutta 4: integrates v data to get x.
% https://en.wikipedia.org/wiki/Runge–Kutta_methods
%
% Inputs: 
%   x1: previous "x" data point
%   v0 - v2: last 3 "v" data points. v2 is the "current" data point.
%   dt: time step between t0 and t1 (or t1 and t2) ("h" in rk eqn)
% Outputs:
%   x2: x data point at time t2.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

k1 = v0;
k2 = v1;
k3 = v1;
k4 = v2;

x2 = x1 + dt/6*(k1+2*k2+2*k3+k4);