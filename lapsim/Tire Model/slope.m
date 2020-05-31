function val = slope(ky,x,r)
%ky50 = movemean(SA(FZ_50_IA_0),FY(FZ_50_IA_0));

%figure(1)
%plot(ky50(:,1),ky50(:,2));
%hold on
ky = polyfit(ky(:,1),ky(:,2),9);
%fplot(@(x) ky(1)*x^9 + ky(2)*x^8 + ky(3)*x^7 + ky(4)*x^6 + ky(5)*x^5 +ky(6)*x^4 +ky(7)*x^3 +ky(8)*x^2+ky(9)*x +ky(10));

syms u
ky = ky(1)*u.^9 + ky(2)*u.^8 + ky(3)*u.^7 + ky(4)*u.^6 + ky(5)*u.^5 +ky(6)*u.^4 +ky(7)*u.^3 +ky(8)*u.^2+ky(9).*u +ky(10);
% ky50 = polyder(polyfit(ky50(:,1),ky50(:,2),9));
% ky100 = polyder(polyfit(ky100(:,1),ky100(:,2),9));
% ky150 = polyder(polyfit(ky150(:,1),ky150(:,2),9));
u = [x-r x x+r];
y = double(subs(ky));
fit = polyfit(u,y,1);
%fplot(@(x) fit(1)*x+fit(2));
val = fit(1);
end
