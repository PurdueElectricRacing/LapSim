function [PC_y, PC_z] = magic_formula_corner(FZ_tire,CA,lambda_mu,load_nom)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Summary: Calculates pacejka tire model coefficients for cornering case
% Inputs: 
%  CA - Camber Angle [degrees]
%  FZ_tire - actual vertical wheel load [N]
%  lambda_mu - friction scaling factor (default = 1)
%  load_nom - nominal wheel load to use (lbs - 50, 100, 150, 200, or 250)
% Outputs:
%  PC_y - [By, Cy, Dy, Ey] (reference Appendix 1 and Chp. 5 of Race Car
%  Design by Derek Seward)
%  PC_z - [Bz, Cz, Dz, Ez] - coefficients for aligning torque
%   Note: aligning torque is dependent on tire pressure.  Here, 12 psi is
%   used.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load 'B1464run22.mat' %cornering tire data
dataset = 22;
Parse_Tire_Data %parse data

switch load_nom  %determine nominal load and variation load indexes
    case num2cell(1:74)
        FZ_nom_i = FZ_50;   %nominal load index 
        FZ_var_i = FZ_100;  %variation with nominal load
        FZ0 = 50*4.448;
        FZ_var = 100*4.448;
    case num2cell(75:124)
        FZ_nom_i = FZ_100;
        FZ_var_i = FZ_50;
        FZ0 = 100*4.448;
        FZ_var = 50*4.448;
    case num2cell(125:174)
        FZ_nom_i = FZ_150;
        FZ_var_i = FZ_50;
        FZ0 = 150*4.448;
        FZ_var = 50*4.448;
    case num2cell(175:224)
        FZ_nom_i = FZ_200;
        FZ_var_i = FZ_50;
        FZ0 = 200*4.448;
        FZ_var = 50*4.448;
    case num2cell(225:351)
        FZ_nom_i = FZ_250;
        FZ_var_i = FZ_50;
        FZ0 = 250*4.448;
        FZ_var = 50*4.448;
    otherwise
        FZ_nom_i = FZ_150; %default case
        FZ_var_i = FZ_50;
        FZ0 = 150*4.448;
        FZ_var = 50*4.448;
end

%get the data points from the indexes and filter
[data_FZ_nom] = movemean(SA( intersect(FZ_nom_i, IA_0) ), FY( intersect(FZ_nom_i, IA_0) ),0.3); % nominal load data with 0 degree camber
[data_IA_4] =   movemean(SA( intersect(FZ_nom_i, IA_4) ), FY( intersect(FZ_nom_i, IA_4) ),0.3); %nominal load data with 4 degree camber
[data_FZ_var] = movemean(SA( intersect(FZ_var_i, IA_0) ), FY( intersect(FZ_var_i, IA_0) ),0.3); %50 lb load data with 0 degree camber

[data_FZ_nom_MZ] = movemean(SA( intersect(intersect(FZ_nom_i, IA_0),P_12) ), MZ( intersect(intersect(FZ_nom_i, IA_0),P_12) ),0.3); 
[data_IA_4_MZ] =   movemean(SA( intersect(intersect(FZ_nom_i, IA_4),P_12) ), MZ( intersect(intersect(FZ_nom_i, IA_4),P_12) ),0.3); 
[data_FZ_var_MZ] = movemean(SA( intersect(intersect(FZ_var_i, IA_0),P_12) ), MZ( intersect(intersect(FZ_var_i, IA_0),P_12) ),0.3); 
% "movemean" is a moving average filtering function

%step 2: pDY1
[Dy_nom,idx] = max(abs(data_FZ_nom(:,2)));
pDY1 = Dy_nom / FZ0; %lateral friction coefficient at nominal load

xm = abs(data_FZ_nom(idx,1))*pi/180; %target slip angle, radians

%step 8: pDY2
del_mu = max(abs(data_FZ_nom(:,2)))/(FZ0)- max(abs(data_FZ_var(:,2)))/(FZ_var); %calc delta from max lateral force friction
del_fz = (FZ0-FZ_var); %calc delta normal load
pDY2 = del_mu/del_fz*FZ0;

%step 9: pDY3
pDY3 = (1 - max(abs(data_IA_4(:,2)))/(FZ0*pDY1) ) / (4*pi/180)^2;

%calculate Dy
dFZ = (FZ_tire-FZ0)/FZ0;
Dy = FZ_tire*(pDY1 + pDY2*dFZ)*(1 - pDY3*(CA*pi/180)^2)*lambda_mu;

%step 4: pCY1 and Cy
kf = 0.7; %correction factor due to data not capturing the real asymptote
ya150 = abs(data_FZ_nom(end,2))*kf; % asymptote of FY/SA curve, nominal load
pCY1 = (1 + (1 - (2/pi)*asin(ya150/Dy_nom)));
Cy = pCY1;
%with the right kf, Cy ~= 1.5 and pEY1 ~= 0

By = slope(data_FZ_nom,0,1)*-180/(pi*Dy*Cy);
pEY1 = (By*xm-tan(pi/(2*pCY1)))/(By*xm-atan(By*xm));
Ey = pEY1;  %zero camber only

PC_y = [By, Cy, Dy, Ey];


% aligning torque coefficients

%step 2: pDY1
[Dy_nom,idx] = max(abs(data_FZ_nom_MZ(:,2)));
pDY1 = Dy_nom / FZ0; %aligning torque "friction" coefficient at nominal load

xm = abs(data_FZ_nom_MZ(idx,1))*pi/180; %indicator slip angle, where aligning torque is maximum

%step 8: pDY2
del_mu = max(abs(data_FZ_nom_MZ(:,2)))/(FZ0)- max(abs(data_FZ_var_MZ(:,2)))/(FZ_var); %calc delta from max lateral force friction
del_fz = (FZ0-FZ_var); %calc delta normal load
pDY2 = del_mu/del_fz*FZ0;

%step 9: pDY3
pDY3 = (1 - max(abs(data_IA_4_MZ(:,2)))/(FZ0*pDY1) ) / (4*pi/180)^2;

%calculate Dy
dFZ = (FZ_tire-FZ0)/FZ0;
Dy = FZ_tire*(pDY1 + pDY2*dFZ)*(1 - pDY3*(CA*pi/180)^2)*lambda_mu;

%step 4: pCY1 and Cy
kf = 1; %correction factor due to data not capturing the real asymptote
ya150 = data_FZ_nom_MZ(end,2)*kf; % asymptote of MZ/SA curve, nominal load
pCY1 = (1 + (1 - (2/pi)*asin(ya150/Dy_nom)));
Cy = pCY1;

By = slope(data_FZ_nom_MZ,0,1)*180/(pi*Dy*Cy);
pEY1 = (By*xm-tan(pi/(2*pCY1)))/(By*xm-atan(By*xm));
Ey = pEY1;  %zero camber only

PC_z = [By, Cy, Dy, Ey];


% figure(2)
% hold on
% plot(data_FZ_nom(:,1)*-pi/180,data_FZ_nom(:,2),".")

% figure(4)
% hold on
% plot(data_FZ_nom_MZ(:,1)*pi/180,data_FZ_nom_MZ(:,2),".")

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