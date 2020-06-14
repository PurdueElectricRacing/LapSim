%create MNC plot

Fz_tire = 9.8*75;
CA = 0;
lambda_mu = 1;

a = -30:0.01:30;
figure(10);
hold on;
for s = [0,0.05,0.1,0.15,0.2,0.3]
     [xx,yy] = MNC(s,a,Fz_tire,CA,lambda_mu);
     figure(10)
     plot(abs(yy),xx,"k.")
end


s = 0:0.001:0.3;
for a = [-30,-20,-10, -5, 0, 5, 10, 20, 30]
    [xx,yy] = MNC(s,a,Fz_tire,CA,lambda_mu);
    figure(10)
    plot(abs(yy),xx,"b.")
end

PC_x = magic_formula_straight(Fz_tire, CA, lambda_mu);
PC_y = magic_formula_corner(Fz_tire, CA, lambda_mu);

%plot the asymptotic value ellipse (kinetic friction)
Fx_inf = PC_x(3)*sin(PC_x(2)*pi/2);
Fy_inf = PC_y(3)*sin(PC_y(2)*pi/2);
t = 0:0.01:pi;
x_inf = Fx_inf*cos(t);
y_inf = Fy_inf*sin(t);
figure(10);
plot(y_inf,x_inf,"r")

%plot the maximum value ellipse (static friction)
Fx_max = PC_x(3);
Fy_max = PC_y(3);
x_max = Fx_max*cos(t);
y_max = Fy_max*sin(t);
plot(y_max,x_max,"g")
