function [out] = magic_formula_straight(FZ_tire,CA,lambda_mu)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Summary: Calculates pacejka tire model coefficients for longitudinal case
% Inputs: 
%  CA - Camber Angle [degrees]
%  FZ_tire - actual vertical wheel load [N]
%  lambda_mu - friction scaling factor (default = 1)
% Outputs:
%  out - [Bx, Cx, Dx, Ex] (reference Appendix 1 and Chp. 5 of Race Car 
% Design by Derek Seward)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load 'B1464run29.mat' %Straight tire data
dataset = 29;
Parse_Tire_Data %parse tire data

%Nominal load for tire model: make sure these two match:
FZ_nom = FZ_250; %nominal load index vector  (choose 150, 200, 250)
FZ0=250*4.448; %Nominal load value for single wheel [N]

[data_FZ_nom] = movemean(SL(intersect(FZ_nom, IA_0)),FX(intersect(FZ_nom, IA_0)),0.01); %nominal load data with 0 degree camber
[data_IA_4] = movemean(SL(intersect(FZ_nom, IA_4)),FX(intersect(FZ_nom, IA_4)),0.01); %nominal load data with 4 degree camber
[data_FZ_50] = movemean(SL(FZ_50_IA_0),FX(FZ_50_IA_0),0.01); %50 lb load data with 0 degree camber
% "movemean" is a moving average filtering function

%step 2: pDY1
[Dx_nom,idx] = max(data_FZ_nom(:,2));
pDY1 = Dx_nom / FZ0; %lateral friction coefficient at nominal load

xm = data_FZ_nom(idx,1); %target slip ratio

%step 8: pDY2
del_mu = max(abs(data_FZ_nom(:,2)))/(FZ0)- max(abs(data_FZ_50(:,2)))/(50*4.448); %calc delta from max lateral force friction
del_fz = (FZ0-50*4.448); %calc delta normal load
pDY2 = del_mu/del_fz*FZ0;

%step 9: pDY3
pDY3 = (1 - max(abs(data_IA_4(:,2)))/(FZ0*pDY1) ) / (4*pi/180)^2;

%calculate Dx
dFZ = (FZ_tire-FZ0)/FZ0;
Dx = FZ_tire*(pDY1 + pDY2*dFZ)*(1 - pDY3*(CA*pi/180)^2)*lambda_mu;

%step 4: pCY1 and Cx
kf = 0.7; %correction factor due to data not capturing the real asymptote
ya150 = abs(data_FZ_nom(end))*kf; % asymptote of FX/SA curve, nominal load
pCY1 = (1 + (1 - (2/pi)*asin(ya150/Dx_nom)));
Cx = pCY1;
%with the right kf, Cx ~= 1.5 and pEY1 ~= 0

Bx = slope(data_FZ_nom,0,0.01)/(Dx*Cx);
pEY1 = (Bx*xm-tan(pi/(2*pCY1)))/(Bx*xm-atan(Bx*xm));
Ex = pEY1;

out = [Bx, Cx, Dx, Ex];

%Print for debugging

    % fprintf('idx: %f\n',idx);
    % fprintf('pDY1: %f\n',pDY1);
    % fprintf('xm: %f\n',xm);
    % fprintf('Dx: %d\n',Dy);
    % fprintf('Bx: %d\n',By);
    % fprintf('Cx: %d\n',Cy);
    % fprintf('Ex: %d\n',Ey);

%plot for debugging
%     figure(1)
%     hold on
%     calc = Dx*sin(Cx*atan(Bx*SL(FZ_50_IA_0)-Ex*(Bx*SL(FZ_50_IA_0)-atan(Bx*SL(FZ_50_IA_0)))));
%     plot(SL(FZ_50_IA_0), calc,'.');
%     plot(data_FZ_150(:,1),data_FZ_150(:,2),'.');
end