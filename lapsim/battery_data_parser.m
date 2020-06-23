
opts = detectImportOptions('VTC6 RPT.csv');
%preview('VTC6 RPT.csv', opts)
opts.SelectedVariableNames = [4;7;9];
opts.DataLines = [58733 58748];
batterytable = readmatrix('VTC6 RPT.csv', opts);
% plot(batterytable(:,2), batterytable(:,3))
% hold on
% plot(batterytable(:,2), batterytable(:,1))
% hold off
%batterytable(:,1) = 100.0015 - (batterytable(:,1) / 29.71);
batterytable(:,3) = batterytable(:,3) / 1000;
batterytable(:,2) = batterytable(:,2) / 1000;
yyaxis left
plot(batterytable(:,1), batterytable(:,2))
yyaxis right
plot(batterytable(:,1), batterytable(:,3))