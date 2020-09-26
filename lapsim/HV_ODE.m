function xp = HV_ODE(t,x,OCV,R,C,Motorload,DCDCload)
xp = zeros(2,1);
vjunction = (OCV+((R(1)/R(3))*x(2))-x(1))/(1+(R(1)/R(3))); % +((R(1)/R(4))*x(3))    +(R(1)/R(4))
Iload = ((vjunction - x(2))/R(3)); % + ((vjunction - x(3))/R(4));
xp(1) = (Iload/C(1)) - (x(1)/(R(2)*C(1)));
xp(2) = (((vjunction - x(2))/R(3))-(Motorload/x(2)))/C(2);
%xp(3) = (((vjunction - x(3))/R(4))-(DCDCload/x(3)))/C(3);