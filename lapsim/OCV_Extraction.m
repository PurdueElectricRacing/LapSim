OCV = xlsread('VTC6 RPT.csv', 'I15084:I18369') / 1000;
SOC = xlsread('VTC6 RPT.csv', 'B15084:B18369');
SOC = 100 - (SOC / 27.38);
curr = -xlsread('VTC6 RPT.csv', 'G15084:G18369') / 1000;
time = xlsread('VTC6 RPT.csv', 'D15084:D18369');
time = time - time(1);
r_table = 
% [t,x] = ode15s(@(t,x) batt_model(t,x,r,c,curr(1)), [time(1),time(2)], 0.0001);