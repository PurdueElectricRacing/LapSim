function x2 = rungekutta(x0, v0, v1, v2, dt)
%%%%%%%%%%%%%%%%%%%%%%%%%
% Use runge-kutta 4: integrates v data to get x.  X data is half the
% resolution of v data - two v data points are required for each x data
% point calculated.
% https://en.wikipedia.org/wiki/Runge–Kutta_methods
%
% Inputs: 
%   x0: previous "x" data point - x(t0)
%   v2: the current value of the variable to be integrated - v(t2)
%   v1: the previous value - v(t1)
%   v0: the previous previous value - v(t0)
%   dt: time step between t0 and t2 ("h" in rk eqn)
% Outputs:
%   x2: x data point at time t2 - x(t2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

k1 = v0;
k2 = v1;
k3 = v1;
k4 = v2;

x2 = x0 + dt/6.*(k1+2*k2+2*k3+k4);

%% Sample code (do not uncomment)
% clear
% close
% 
% last = 200; %number of data points
% n = 0:last;
% h = 1/20; %time step
% t = n*h;  %time
% a = sin(t)-0.2; %actual acceleration
% v = -cos(t)-0.2*t+1; %actual velocity
% x = -sin(t)-0.1*t.^2+t; %actual displacement
% 
% plot(t,a,"k.");
% hold on;
% plot(t,v,"r");
% plot(t,x,"b");
% 
% %calculate velocity using runge-kutta
% vrk = zeros(last/2+1,1);
% n2 = 1:1:last/2+1;
% for i = n2(1:end-1)
%     vrk(i+1) = rungekutta(vrk(i), a(2*i-1), a(2*i), a(2*i+1), 2*h);
% end
% plot((n2-1)*2*h,vrk,"r.")
% 
% %calculate displacement using runge-kutta
% xrk = zeros(last/4+1,1);
% n4 = 1:1:last/4+1;
% for i = n4(1:end-1)
%     xrk(i+1) = rungekutta(xrk(i), vrk(2*i-1), vrk(2*i), vrk(2*i+1), 4*h);
% end
% plot((n4-1)*4*h,xrk,"b.")
