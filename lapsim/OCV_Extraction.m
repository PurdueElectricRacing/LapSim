OCV = xlsread('VTC6 RPT.csv', 'I15084:I18650') / 1000;
SOC = xlsread('VTC6 RPT.csv', 'B15084:B18650');
SOC = 100 - (SOC / 29.71042);
curr = -xlsread('VTC6 RPT.csv', 'G15084:G18650') / 1000;
time = xlsread('VTC6 RPT.csv', 'D15084:D18650');
time = time - time(1);
rc_table = xlsread('VTC6_Data.csv', 'O4:R12'); 
r(1) = interp1(rc_table(:,1), rc_table(:,2), SOC(1), 'makima') / 1000;
r(2) = interp1(rc_table(:,1), rc_table(:,3), SOC(1), 'makima') / 1000;
c = interp1(rc_table(:,1), rc_table(:,4), SOC(1), 'makima');
[t,x] = ode15s(@(t,x) batt_model(t,x,r(2),c,curr(1)), [time(1),time(2)], 0.0001);
detail = x + curr(1) * r(1) + OCV(1);
ocv_n = [100 4.2;SOC(1) detail(end)];
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
ocv_n = [ocv_n; 0 2.5];
ocv_n(2:2:end,:) = []; %cut down samples if necessary
%xlswrite('temp.xlsx', ocv_n, 1, 'L4'); %for writing results to excel
%sheet (be careful with this one)