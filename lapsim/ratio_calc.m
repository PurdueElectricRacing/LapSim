close all
clear
clc

config; %loads config
autox; %sets track table
step = .1; % step size
L = 1;

for x = [1:step:10]
    fdr = x;
    run_track;
    
    max_speed(L) = max(velocity);
    avg_speed(L) = mean(velocity);
    max_accel(L) = max(acceleration);
    L = L + 1;
    
end

lx = [1:step:x];
figure('Name', 'Max Speed')
plot(lx, max_speed)

figure('Name', 'Avg Speed')
plot(lx, avg_speed)

figure('Name', 'Max Accel')
plot(lx, max_accel)


% ass = 1;
% ideal_s = zeros(1,100);
% ideal_f = zeros(1,15);
% for x = [0.8:step:2.2]
%    coeff_f = x;
%    for y = [1:step:10]
%      fdr = y; 
%      run_track;
%      if ideal_s(ass) <= max(velocity)
%         ideal_f(ass) = y;
%         ideal_s(ass) = max(velocity);
%         
%      end
%    end
%   ass = ass + 1;
% end
% 
% lx = [.8:step:2.2];
% 
% 
% 
% figure('Name', 'Max Accel')
% plot(lx, ideal_f)