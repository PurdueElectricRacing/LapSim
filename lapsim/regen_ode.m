function xp = regen_ode(t,x,R,C,OCV, Iload, lvload)
xp = zeros(2,1);
vbat = OCV - (Iload * R(1)) - x(1);
xp(1) = (Iload / C(1)) - (x(1) / (R(1) * C(1)));
xp(2) = (((vbat - x(2))/R(4))-(lvload/x(2)))/C(3);