
%create MNC plot

Fz_tire = 9.8*75;
CA = 0;
lambda_mu = 1;

figure(10); %set up friction ellipse figure
ylim([-1500,1500]); %change these ranges if neccesary
xlim([0,2000]);
hold on;

a = -30:0.01:30; %range of slip angles
for s = [0.05,0.1,0.15,0.2,0.3] % plot these slip ratios for the range of slip angles
    [xx,yy] = MNC(s,a,Fz_tire,CA,lambda_mu);
    figure(10)
    plot(abs(yy),xx,"k.")
    %fprintf("SR = %f\n",s);   %uncomment for one at a time plotting
    %input("hit enter")
end

s = 0:0.001:0.3; %range of slip ratios
for a = [5, 10, 20, 30]  %plot these slip angles for the range of slip ratios
    [xx,yy] = MNC(s,a,Fz_tire,CA,lambda_mu);
    figure(10)
    plot(abs(yy),xx,"b.")
    plot(abs(yy),-xx,"b.")
    %fprintf("SA = %f\n",a);
    %input("hit enter")
end

%get pacejka coefficients again
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

title("Tire-Force Ellipse")
xlabel("Fy(s,a) (N)")
ylabel("Fx(s,a) (N)")

%create 3D plot
results = []; %combine results into one array: s, a, fx, fy

a = -30.01:1:30.01; % note MNC is undefined when s or a = 0
s = -0.3001:0.01:0.3001;
for sr = s % plot these slip ratios for the range of slip angles
    [xx,yy] = MNC(sr,a,Fz_tire,CA,lambda_mu);
    results = [results; sr*ones(length(a),1), a', xx', yy'];
end

%3D plot of force vs slip ratio and angle
figure(20);
total_force = sqrt(results(:,3).^2 + results(:,4).^2);
%plot3(results(:,1),results(:,2),total_force,".")

fx_sr = calc_magic_formula(PC_x,s);
fy_sa = calc_magic_formula(PC_y,a*pi/180);
%plot3(zeros(length(a),1),a,abs(fy_sa),".");
%hold on
%plot3(s,zeros(length(s),1),abs(fx_sr),".");

%MNC is undefined at s or a = 0
%total_force(find(results(:,1) == 0)) = abs(fy_sa);
%total_force(find(results(:,2) == 0)) = abs(fx_sr);
total_force = reshape(total_force, [length(s), length(a)]);
surf(s,a,total_force,'FaceColor','interp');
zlim([1000,2000])
title("3D Pacejka curve using MNC model")
xlabel("slip ratio")
ylabel("slip angle")
zlabel("max traction")
