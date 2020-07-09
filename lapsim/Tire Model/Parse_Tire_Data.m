%Parse loaded tire data

%all vectors created here are index vectors for the raw tire data
%example: FY(FZ_100_IA_1) for all Fy for a load of 100 lbs and inclination
%of 1


%uncomment one of these if not calling from a function:

% load 'B1464run22.mat' %cornering tire data
% dataset = 22;

% load 'B1464run29.mat' %straight tire data
% dataset = 29;


%get the correct start time for the given dataset
%define dataset first
start_time_list = [22, 400; %dataset 22 starts at ET = 400
                   29, 0];   
start_time = start_time_list(find(start_time_list(:,1) == dataset),2);

IA_0 = find(IA>-.1 & IA<=0.1); %index matrix for 0 degree inclination angle
IA_1 = find(IA>0.9 & IA<=1.1); %index matrix for 1 degree inclination angle
IA_2 = find(IA>1.9 & IA<=2.1); %index matrix for 2 degree inclination angle
IA_3 = find(IA>2.9 & IA<=3.1); %index matrix for 3 degree inclination angle
IA_4 = find(IA>3.9 & IA<=4.9); %index matrix for 4 degree inclination angle

FZ_50 = find(FZ<-180 & FZ>-267 & ET>start_time); %index matrix for 50lb normal load
FZ_100 = find(FZ<-356 & FZ>-534 & ET>start_time); %index matrix for 100lb normal load
FZ_150 = find(FZ<-578 & FZ>-756 & ET>start_time); %index matrix for 150lb normal load
FZ_200 = find(FZ<-801 & FZ>-979 & ET>start_time); %index matrix for 200lb normal load
FZ_250 = find(FZ<-1023 & FZ>-1201 & ET>start_time); %index matrix for 250lb normal load

FZ_50_IA_0 = intersect(FZ_50, IA_0);
FZ_100_IA_0 = intersect(FZ_100, IA_0);
FZ_150_IA_0 = intersect(FZ_150, IA_0);
FZ_200_IA_0 = intersect(FZ_200, IA_0);
FZ_250_IA_0 = intersect(FZ_250, IA_0);

FZ_50_IA_1 = intersect(FZ_50, IA_1);
FZ_100_IA_1 = intersect(FZ_100, IA_1);
FZ_150_IA_1 = intersect(FZ_150, IA_1);
FZ_200_IA_1 = intersect(FZ_200, IA_1);
FZ_250_IA_1 = intersect(FZ_250, IA_1);

FZ_50_IA_2 = intersect(FZ_50, IA_2);
FZ_100_IA_2 = intersect(FZ_100, IA_2);
FZ_150_IA_2 = intersect(FZ_150, IA_2);
FZ_200_IA_2 = intersect(FZ_200, IA_2);
FZ_250_IA_2 = intersect(FZ_250, IA_2);

FZ_50_IA_3 = intersect(FZ_50, IA_3);
FZ_100_IA_3 = intersect(FZ_100, IA_3);
FZ_150_IA_3 = intersect(FZ_150, IA_3);
FZ_200_IA_3 = intersect(FZ_200, IA_3);
FZ_250_IA_3 = intersect(FZ_250, IA_3);

FZ_50_IA_4 = intersect(FZ_50, IA_4);
FZ_100_IA_4 = intersect(FZ_100, IA_4);
FZ_150_IA_4 = intersect(FZ_150, IA_4);
FZ_200_IA_4 = intersect(FZ_200, IA_4);
FZ_250_IA_4 = intersect(FZ_250, IA_4);

%separate out tire pressures - kpa
P_10 = find(P < 74); %10 psi
P_12 = find(P > 75 & P < 88); %12 psi
P_14 = find(P > 89); %14 psi


% plot(-pi/180*SA(FZ_50_IA_0), NFY(FZ_50_IA_0),"b.")
% hold on
% plot(-pi/180*SA(FZ_100_IA_0), NFY(FZ_100_IA_0),"k.")
% plot(-pi/180*SA(FZ_150_IA_0), NFY(FZ_150_IA_0),"r.")
% plot(-pi/180*SA(FZ_200_IA_0), NFY(FZ_200_IA_0),"g.")
% plot(-pi/180*SA(FZ_250_IA_0), NFY(FZ_250_IA_0),"m.")
 
% plot(pi/180*SA(intersect(FZ_50_IA_4,P_12)), MZ(intersect(FZ_50_IA_4,P_12)),"b.")
% hold on
% plot(pi/180*SA(intersect(FZ_100_IA_4,P_12)), MZ(intersect(FZ_100_IA_4,P_12)),"k.")
% plot(pi/180*SA(intersect(FZ_150_IA_4,P_12)), MZ(intersect(FZ_150_IA_4,P_12)),"r.")
% plot(pi/180*SA(intersect(FZ_200_IA_4,P_12)), MZ(intersect(FZ_200_IA_4,P_12)),"g.")
% plot(pi/180*SA(intersect(FZ_250_IA_4,P_12)), MZ(intersect(FZ_250_IA_4,P_12)),"m.")

% plot(SL(FZ_50_IA_0), FX(FZ_50_IA_0),"b.")
% hold on
% plot(SL(FZ_100_IA_0), FX(FZ_100_IA_0),"k.")
% plot(SL(FZ_150_IA_0), FX(FZ_150_IA_0),"r.")
% plot(SL(FZ_200_IA_0), FX(FZ_200_IA_0),"g.")
% plot(SL(FZ_250_IA_0), FX(FZ_250_IA_0),"m.")

% % figure(1)
% % plot(ET(FZ_50), SA(FZ_50),'.')
% % figure(2)
% % plot(ET(FZ_50), FY(FZ_50),'.')
% ky50 = [SA(FZ_50_IA_0),FY(FZ_50_IA_0)];
% [~,idx] = sort(ky50(:,1));
% ky50_sort = ky50(idx,:);
% A = movmean(ky50_sort, 100);
% %plot(A(:,1),A(:,2),".")