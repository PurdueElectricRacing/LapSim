function [out] = magic_formula_straight(FZ_tire,CA,lambda_mu,load_nom)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Summary: Calculates pacejka tire model coefficients for longitudinal case
% Inputs: 
%  CA - Camber Angle [degrees]
%  FZ_tire - actual vertical wheel load [N]
%  lambda_mu - friction scaling factor (default = 1)
%  load_nom - nominal wheel load to use (lbs - 50, 150, 200, or 250)
% Outputs:
%  out - [Bx, Cx, Dx, Ex] (reference Appendix 1 and Chp. 5 of Race Car 
% Design by Derek Seward)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load 'B1464run29.mat' %Straight tire data
dataset = 29;
Parse_Tire_Data %parse tire data

switch load_nom  %determine nominal load and variation load indexes
    case num2cell(1:74)
        FZ_nom_i = FZ_50;   %nominal load index 
        FZ_var_i = FZ_150;  %variation with nominal load
        FZ0 = 50*4.448;
        FZ_var = 150*4.448;
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
[data_FZ_nom] = movemean(SL(intersect(FZ_nom_i, IA_0)),FX(intersect(FZ_nom_i, IA_0)),0.01); %nominal load data with 0 degree camber
[data_IA_4] = movemean(SL(intersect(FZ_nom_i, IA_4)),FX(intersect(FZ_nom_i, IA_4)),0.01); %nominal load data with 4 degree camber
[data_FZ_var] = movemean(SL(intersect(FZ_var_i, IA_0)),FX(intersect(FZ_var_i, IA_0)),0.01); %50 lb load data with 0 degree camber
% "movemean" is a moving average filtering function

%vertical offset correction 
FX_offset = mean(data_FZ_nom(:,2));
data_FZ_nom(:,2) = data_FZ_nom(:,2) - FX_offset;
FX_offset = mean(data_FZ_var(:,2));
data_FZ_var(:,2) = data_FZ_var(:,2) - FX_offset;

%step 2: pDY1
Dx_nom = (max(data_FZ_nom(:,2)) - min(data_FZ_nom(:,2)))./2;
[~,idx] = max(data_FZ_nom(:,2));
pDY1 = Dx_nom / FZ0; %lateral friction coefficient at nominal load

xm = abs(data_FZ_nom(idx,1)); %target slip ratio

%step 8: pDY2
del_mu = max(abs(data_FZ_nom(:,2)))/(FZ0)- max(abs(data_FZ_var(:,2)))/(FZ_var); %calc delta from max lateral force friction
del_fz = (FZ0-FZ_var); %calc delta normal load
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

Bx = slope(data_FZ_nom,0,0.03)/(Dx*Cx);
pEY1 = (Bx*xm-tan(pi/(2*pCY1)))/(Bx*xm-atan(Bx*xm));
Ex = pEY1;

out = [Bx, Cx, Dx, Ex];

% figure(3)
% hold on
% plot(data_FZ_nom(:,1),data_FZ_nom(:,2),".")

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