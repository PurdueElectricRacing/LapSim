clear
clc

config; %loads config
autox; %sets track table
step = 0.1; % step size
i = 0;

for x = [1:step:5]
    fdr = x;
    run_track;
    
    max_speed(i) = max(velocity);
    avg_speed(i) = mean(velocity);
    
end

fdr = [1:step:x];

figure('Name', 'Max Speed')
plot(fdr, max_speed)

figure('Name', 'Avg Speed')
plot(fdr, avg_spped)