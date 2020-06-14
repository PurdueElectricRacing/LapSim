function [out] = magic_formula_corner(FZ_tire,CA,lambda_mu)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Summary: Calculates pacejka tire model coefficients for cornering case
% Inputs: 
%  CA - Camber Angle [degrees]
%  FZ_tire - actual vertical wheel load [N]
%  lambda_mu - friction scaling factor (default = 1)
% Outputs:
%  out - [By, Cy, Dy, Ey] (reference Appendix 1 and Chp. 5 of Race Car
%  Design by Derek Seward)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load 'B1464run22.mat' %cornering tire data
Parse_Tire_Data %parse data
    
FZ0=150*4.448; %Nominal load value for single wheel [N]
[data_FZ_150] = movemean(SA(FZ_150_IA_0),FY(FZ_150_IA_0),0.3); % nominal load data with 0 degree camber
[data_IA_4] = movemean(SA(FZ_150_IA_4),FY(FZ_150_IA_4),0.3); %nominal load data with 4 degree camber
[data_FZ_50] = movemean(SA(FZ_50_IA_0),FY(FZ_50_IA_0),0.3); %50 lb load data with 0 degree camber
% "movemean" is a moving average filtering function

%step 2: pDY1
[pDY1,idx] = max(abs(data_FZ_150(:,2)));
pDY1 = pDY1 / FZ0; %lateral friction coefficient at nominal load

xm = abs(data_FZ_150(idx,1))*pi/180; %target slip angle, radians

%step 8: pDY2
del_mu = max(abs(data_FZ_150(:,2)))/(FZ0)- max(abs(data_FZ_50(:,2)))/(50*4.448); %calc delta from max lateral force friction
del_fz = (FZ0-50*4.448); %calc delta normal load
pDY2 = del_mu/del_fz*FZ0;

%step 9: pDY3
pDY3 = (1 - max(abs(data_IA_4(:,2)))/(FZ0*pDY1) ) / (4*pi/180)^2;

%calculate Dy
dFZ = (FZ_tire-FZ0)/FZ0;
Dy = FZ_tire*(pDY1 + pDY2*dFZ)*(1 - pDY3*(CA*pi/180)^2)*lambda_mu;

%step 4: pCY1 and Cy
kf = 0.7; %correction factor due to data not capturing the real asymptote
ya150 = abs(data_FZ_150(end))*kf; % asymptote of FY/SA curve, nominal load
pCY1 = (1 + (1 - (2/pi)*asin(ya150/Dy)));
Cy = pCY1;
%with the right kf, Cy ~= 1.5 and pEY1 ~= 0

By = slope(data_FZ_150,0,1)*-180/(pi*Dy*Cy);
pEY1 = (By*xm-tan(pi/(2*pCY1)))/(By*xm-atan(By*xm));
Ey = pEY1;  %zero camber only

out = [By, Cy, Dy, Ey];

%Print for debugging

    % fprintf('idx: %f\n',idx);
    % fprintf('pDY1: %f\n',pDY1);
    % fprintf('xm: %f\n',xm);
    % fprintf('Dx: %d\n',Dy);
    % fprintf('Bx: %d\n',By);
    % fprintf('Cx: %d\n',Cy);
    % fprintf('Ex: %d\n',Ey);

%plot for debugging
%     figure(2);
%     hold on
%     calc = -Dy*sin(Cy*atan(By*SA(FZ_150_IA_0)*pi/180-Ey*(By*SA(FZ_150_IA_0)*pi/180-atan(By*SA(FZ_150_IA_0)*pi/180))));
%     plot(SA(FZ_150_IA_0),calc, '.')
%     plot(data_FZ_150(:,1),data_FZ_150(:,2),'.');
    
end