function [By, Cy, Dy, Ey] = magic_formula(nom,CA)
%%
% I believe this function is unused, but I might be wrong
% - Shanks


load 'B1464run22.mat' %cornering tire data
Parse_Tire_Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%select nominal load value
%if nom >= 150*4.448
    FZ0=150*4.448;
    [val] = movemean(SA(FZ_150_IA_0),FY(FZ_150_IA_0),0.3);
    [val_IA_4] = movemean(SA(FZ_150_IA_4),FY(FZ_150_IA_4),0.3);
    [val2] = movemean(SA(FZ_50_IA_0),FY(FZ_50_IA_0),0.3);
% elseif nom <= 50*4.448
%     FZ0=50*4.448;
%     [val] = movemean(SA(FZ_50_IA_0),FY(FZ_50_IA_0));
%     [val_IA_4] = movemean(SA(FZ_50_IA_4),FY(FZ_50_IA_4));
% elseif (nom/(100*4.448)) > 1
%     if nom-100*4.448 < nom-150*4.448
%         FZ0 = 100*4.448;
%         [val] = movemean(SA(FZ_100_IA_0),FY(FZ_100_IA_0));
%         [val_IA_4] = movemean(SA(FZ_100_IA_4),FY(FZ_100_IA_4));
%     else
%         FZ0 = 150*4.448;
%         [val] = movemean(SA(FZ_150_IA_0),FY(FZ_150_IA_0));
%         [val_IA_4] = movemean(SA(FZ_150_IA_4),FY(FZ_150_IA_4));
%     end
% else
%     if nom-100*4.448 < nom-50*4.448
%          FZ0 = 100*4.448;
%          [val] = movemean(SA(FZ_100_IA_0),FY(FZ_100_IA_0));
%          [val_IA_4] = movemean(SA(FZ_100_IA_4),FY(FZ_100_IA_4));
%     else 
%          FZ0 = 50*4.448;
%          [val] = movemean(SA(FZ_50_IA_0),FY(FZ_50_IA_0));
%          [val_IA_4] = movemean(SA(FZ_50_IA_4),FY(FZ_50_IA_4));
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pDY1 = max(abs(val(:,2)))/FZ0;
idx = find(abs(val(:,2))==pDY1*FZ0);
xm = val(idx,1)*pi/180;
del_mu = max(abs(val(:,2)))/(FZ0)- max(abs(val2(:,2)))/(50*4.448); %calc delta from max lateral force friction
del_fz = (FZ0-50*4.448);
pDY2 = del_mu/del_fz*FZ0;

pDY3 = (1- max(abs(val_IA_4(:,2)))/(FZ0*pDY1))/(4*pi/180)^2;
Dy = nom*(pDY1+pDY2*(nom-FZ0)/150)*(1-pDY3*(CA*pi/180)^2);


ya150 = abs(val(end));
pCY1 = (1 + (1 - (2/pi)*asin(ya150/Dy*pi/180)));
Cy = pCY1;

By = slope(val,0)*-180/(pi*Dy*Cy);
fprintf("By: %d\n",By);
pEY1 = (By*xm-tan(pi/(2*pCY1)))/(By*xm-atan(By*xm));
Ey = pEY1;

calc = -Dy*sin(Cy*atan(By*SA(FZ_50_IA_0)*pi/180-Ey*(By*SA(FZ_50_IA_0)*pi/180-atan(By*SA(FZ_50_IA_0)*pi/180))));

end