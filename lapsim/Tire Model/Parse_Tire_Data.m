%Parse loaded tire data

IA_0 = find(IA>-.1 & IA<=0.1); %index matrix for 0 degree inclination angle
IA_1 = find(IA>0.9 & IA<=1.1); %index matrix for 1 degree inclination angle
IA_2 = find(IA>1.9 & IA<=2.1); %index matrix for 2 degree inclination angle
IA_3 = find(IA>2.9 & IA<=3.1); %index matrix for 3 degree inclination angle
IA_4 = find(IA>3.9 & IA<=4.9); %index matrix for 4 degree inclination angle

FZ_50 = find(FZ<-180 & FZ>-260 & ET>520); %index matrix for 50lb normal load
FZ_100 = find(FZ<-400 & FZ>-480 & ET>520); %index matrix for 100lb normal load
FZ_150 = find(FZ<-620 & FZ>-710 & ET>520); %index matrix for 150lb normal load
FZ_250 = find(FZ<-1090 & FZ>-1150 & ET>520); %index matrix for 250lb normal load
FZ_350 = find(FZ<-1450 & FZ>-1600); %index matrix for 350lb normal load

FZ_50_IA_0 = intersect(FZ_50, IA_0);
FZ_100_IA_0 = intersect(FZ_100, IA_0);
FZ_150_IA_0 = intersect(FZ_150, IA_0);
FZ_250_IA_0 = intersect(FZ_250, IA_0);
FZ_350_IA_0 = intersect(FZ_350, IA_0);

FZ_50_IA_1 = intersect(FZ_50, IA_1);
FZ_100_IA_1 = intersect(FZ_100, IA_1);
FZ_150_IA_1 = intersect(FZ_150, IA_1);
FZ_250_IA_1 = intersect(FZ_250, IA_1);
FZ_350_IA_1 = intersect(FZ_350, IA_1);

FZ_50_IA_2 = intersect(FZ_50, IA_2);
FZ_100_IA_2 = intersect(FZ_100, IA_2);
FZ_150_IA_2 = intersect(FZ_150, IA_2);
FZ_250_IA_2 = intersect(FZ_250, IA_2);
FZ_350_IA_2 = intersect(FZ_350, IA_2);

FZ_50_IA_3 = intersect(FZ_50, IA_3);
FZ_100_IA_3 = intersect(FZ_100, IA_3);
FZ_150_IA_3 = intersect(FZ_150, IA_3);
FZ_250_IA_3 = intersect(FZ_250, IA_3);
FZ_350_IA_3 = intersect(FZ_350, IA_3);

FZ_50_IA_4 = intersect(FZ_50, IA_4);
FZ_100_IA_4 = intersect(FZ_100, IA_4);
FZ_150_IA_4 = intersect(FZ_150, IA_4);
FZ_250_IA_4 = intersect(FZ_250, IA_4);
FZ_350_IA_4 = intersect(FZ_350, IA_4);

% % figure(1)
% % plot(ET(FZ_50), SA(FZ_50),'.')
% % figure(2)
% % plot(ET(FZ_50), FY(FZ_50),'.')
% ky50 = [SA(FZ_50_IA_0),FY(FZ_50_IA_0)];
% [~,idx] = sort(ky50(:,1));
% ky50_sort = ky50(idx,:);
% A = movmean(ky50_sort, 100);
% %plot(A(:,1),A(:,2),".")