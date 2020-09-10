OCV = xlsread('VTC6 RPT.csv', 'I57150:I58294') / 1000;
SOC = xlsread('VTC6 RPT.csv', 'B57150:B58294');
SOC = (SOC / 28.597);
% SOC(1:2:end) = [];
% OCV(1:2:end) = [];
xlswrite('VTC6_Data.xlsx', [SOC,OCV], 1, 'F4');
plot(SOC,OCV);
% v = xlsread('VTC6 RPT.csv', 'I67878:I68239') / 1000;
% t = xlsread('VTC6 RPT.csv', 'D67879:D68239');
% t = t - t(1);
% SOC_C = interp1(OCV, SOC, v(end));
% sr = (v(1) - v(2)) / 0.9; %charging mode
% v = v(2:end);
% dif = (max(v) - min(v));
% r = dif / 0.9;
% c = 100:20000;
% corr = 0;
% for i = c(1):c(end)
%     x = min(v) + (dif * exp(-t / (r * i))); %charging mode
%     R = abs(x - v) ./ v;
%     R = mean(R) * 100;
%     R = 1 - R;
%     d(i - c(1) + 1) = R;
%     if R > corr; corr = R; cap = i; end
% end
% disp(['SOC ',num2str(SOC_C)])
% disp(['Static R ', num2str(sr)])
% disp(['Dynamic R ', num2str(r)])
% disp(['Correlation ',num2str(corr)])
% disp(['Cap Value ',num2str(cap),' Farads'])
% %plot(t,v,t,x)
% 
