function xp = batt_model(t,x,r,c,Iload)
xp = zeros(1);
xp(1) = (Iload / c) - (x(1) / (r * c));