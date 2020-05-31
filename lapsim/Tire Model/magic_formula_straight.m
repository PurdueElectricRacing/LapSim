function [out] = magic_formula_straight(nom,CA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Summary: Calculates pacejka tire model coefficients for cornering case
%Inputs: Camber Angle [degrees]
%Outputs Bx, Cx, Dx, Ex [refrence Appendix 1 of Race Car Design by Derek Seward]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


load 'B1464run29.mat' %Straight tire data
Parse_Tire_Data %parse tire data

    FZ0=150*4.448; %Nominal load value for single wheel [N]
    [val] = movemean(SL(FZ_150_IA_0),FX(FZ_150_IA_0),0.01); %nominal load data with 0 degree camber
    [val_IA_4] = movemean(SL(FZ_150_IA_4),FX(FZ_150_IA_4),0.01); %nominal load data with 4 degree camber
    [val2] = movemean(SL(FZ_50_IA_0),FX(FZ_50_IA_0),0.01); %50 lb load data with 0 degree camber

idx = find(max(abs(val(:,2)))); %index variable
pDY1 = abs(val(idx,2))/FZ0;
xm = abs(val(idx,1));
del_mu = max(abs(val(:,2)))/(FZ0)- max(abs(val2(:,2)))/(50*4.448); %calc delta from max lateral force friction
del_fz = (FZ0-50*4.448);
pDY2 = del_mu/del_fz*FZ0;

pDY3 = (1- max(abs(val_IA_4(:,2)))/(FZ0*pDY1))/(4*pi/180)^2;
Dx = nom*(pDY1+pDY2*(nom-FZ0)/150)*(1-pDY3*(CA*pi/180)^2);


ya150 = abs(val(end));
pCY1 = (1 + (1 - (2/pi)*asin(ya150/Dx*pi/180)));
Cx = pCY1;

Bx = slope(val,0,0.01)/(Dx*Cx);
pEY1 = (Bx*xm-tan(pi/(2*pCY1)))/(Bx*xm-atan(Bx*xm));
Ex = pEY1;

out = [Bx, Cx, Dx, Ex];

%Print for debugging

    % fprintf('idx: %f\n',idx);
    fprintf('pDY1: %f\n',pDY1);
    % fprintf('xm: %f\n',xm);
    % fprintf('Dx: %d\n',Dy);
    % fprintf('Bx: %d\n',By);
    % fprintf('Cx: %d\n',Cy);
    % fprintf('Ex: %d\n',Ey);

%plot for debugging

    %calc = Dy*sin(Cy*atan(By*SL(FZ_50_IA_0)-Ey*(By*SL(FZ_50_IA_0)-atan(By*SL(FZ_50_IA_0)))));
    %plot(SL(FZ_50_IA_0), calc,'.');
    %hold on
    %plot(val(:,1),val(:,2),'.');
end