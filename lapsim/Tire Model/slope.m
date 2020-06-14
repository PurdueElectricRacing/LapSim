function val = slope(ky,x,r)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Summary: Calculates the slope of a data set near a point
% Inputs: 
%  ky - (x,y) data
%  x - center point of slope calculation
%  r - range of slope calculation
% Outputs:
%  val - 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ky = polyfit(ky(:,1),ky(:,2),9); %9th degree polynomial
%fplot(@(x) ky(1)*x^9 + ky(2)*x^8 + ky(3)*x^7 + ky(4)*x^6 + ky(5)*x^5 +ky(6)*x^4 +ky(7)*x^3 +ky(8)*x^2+ky(9)*x +ky(10));

syms u
ky = ky(1)*u.^9 + ky(2)*u.^8 + ky(3)*u.^7 + ky(4)*u.^6 + ky(5)*u.^5 +ky(6)*u.^4 +ky(7)*u.^3 +ky(8)*u.^2+ky(9).*u +ky(10);
u = [x-r x x+r];
y = double(subs(ky));
fit = polyfit(u,y,1);
%hold on;
%fplot(@(x) fit(1)*x+fit(2));
val = fit(1);

%alternatively, if syms is not available,
%ky = polyfit(ky(:,1),ky(:,2),9);
%val = ky(9);
end
