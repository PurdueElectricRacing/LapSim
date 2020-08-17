OCV = xlsread('VTC6 RPT.csv', 'I57150:I58294') / 1000; %this is currently set up for charging data
SOC = xlsread('VTC6 RPT.csv', 'B57150:B58294');
SOC = (SOC / 28.597);
curr = -xlsread('VTC6 RPT.csv', 'G57150:G58294') / 1000;
time = xlsread('VTC6 RPT.csv', 'D57150:D58294');
time = time - time(1);
rc_table = xlsread('VTC6_Data.xlsx', 'W4:Z13'); 
r(1) = interp1(rc_table(:,1), rc_table(:,2), SOC(1), 'makima') / 1000;
r(2) = interp1(rc_table(:,1), rc_table(:,3), SOC(1), 'makima') / 1000;
c = interp1(rc_table(:,1), rc_table(:,4), SOC(1), 'makima');
[t,x] = ode15s(@(t,x) batt_model(t,x,r(2),c,curr(1)), [time(1),time(2)], 0.0001);
detail = x + curr(1) * r(1) + OCV(1);
ocv_n = [0 3; SOC(1) detail(end)];
xo = x(end);
for i = 2:size(SOC) - 1
    r(1) = interp1(rc_table(:,1), rc_table(:,2), SOC(i), 'makima', 'extrap') / 1000;
    r(2) = interp1(rc_table(:,1), rc_table(:,3), SOC(i), 'makima', 'extrap') / 1000;
    c = interp1(rc_table(:,1), rc_table(:,4), SOC(i), 'makima', 'extrap');
    [t,x] = ode15s(@(t,x) batt_model(t,x,r(2),c,curr(i)), [time(i),time(i+1)], xo);
    detail = x + curr(i) * r(1) + OCV(i);
    ocv_n = [ocv_n; SOC(i) detail(end)];
    xo = x(end);
end
ocv_n = [ocv_n;100 4.2];
%ocv_n(2:2:end,:) = []; %cut down samples if necessary
%xlswrite('VTC6_Data.xlsx', ocv_n, 1, 'L4'); %for writing results to excel
%sheet (be careful with this one)