function [out] = magic_formula_corner(nom,CA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Summary: Calculates pacejka tire model coefficients for cornering case
%Inputs: Camber Angle [degrees]
%Outputs By, Cy, Dy, Ey [refrence Appendix 1 of Race Car Design by Derek Seward]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load 'B1464run22.mat' %cornering tire data
Parse_Tire_Data %parse data
    
    FZ0=150*4.448; %Nominal load value for single wheel [N]
    [val] = movemean(SA(FZ_150_IA_0),FY(FZ_150_IA_0),0.3); % nominal load data with 0 degree camber
    [val_IA_4] = movemean(SA(FZ_150_IA_4),FY(FZ_150_IA_4),0.3); %nominal load data with 4 degree camber
    [val2] = movemean(SA(FZ_50_IA_0),FY(FZ_50_IA_0),0.3); %50 lb load data with 0 degree camber


pDY1 = max(abs(val(:,2)))/FZ0;
idx = find(abs(val(:,2))==pDY1*FZ0); %index variable

xm = val(idx,1)*pi/180;

del_mu = max(abs(val(:,2)))/(FZ0)- max(abs(val2(:,2)))/(50*4.448); %calc delta from max lateral force friction
del_fz = (FZ0-50*4.448); %calc delta normal load

pDY2 = del_mu/del_fz*FZ0;
pDY3 = (1- max(abs(val_IA_4(:,2)))/(FZ0*pDY1))/(4*pi/180)^2;
Dy = nom*(pDY1+pDY2*(nom-FZ0)/150)*(1-pDY3*(CA*pi/180)^2);


ya150 = abs(val(end)); % max offset
pCY1 = (1 + (1 - (2/pi)*asin(ya150/Dy*pi/180)));
Cy = pCY1;

By = slope(val,0,1)*-180/(pi*Dy*Cy);
pEY1 = (By*xm-tan(pi/(2*pCY1)))/(By*xm-atan(By*xm));
Ey = pEY1;

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

    %calc = -Dy*sin(Cy*atan(By*SA(FZ_50_IA_0)*pi/180-Ey*(By*SA(FZ_50_IA_0)*pi/180-atan(By*SA(FZ_50_IA_0)*pi/180))));
    %plot(SA(FZ_50_IA_0),calc, '.')
    %hold on
    %plot(val(:,1),val(:,2),'.');
end