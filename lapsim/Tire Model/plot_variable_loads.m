% test script - variable loads are calculated using the pacejka method and
% by interpolating the data set.

Fx = [];
PC = [];
for L = 4.448*[50,100,150,200,250]
    PC = [PC; magic_formula_corner(L, 0, 1,150)];
    for s = -0.3:0.01:0.3
        Fx = [Fx,calc_magic_formula(PC(end,:),s)];
    end
    figure(2);
    plot((-0.3:0.01:0.3), Fx, "k");
    hold on;
    Fx = [];
end
title("load scaling using variation coefficient pDY2 (y)")

test_tire = tire();  % create instance of tire class
x_data_loads = [50 150 200 250]*4.448;
y_data_loads = [50 100 150 200 250]*4.448;

for xl = x_data_loads
    test_tire.interp_PCs(xl);
    test_tire.plot_magic_formula('x');
end

for yl = y_data_loads
    test_tire.interp_PCs(yl);
    test_tire.plot_magic_formula('y');
end

figure(2)
title("load scaling using interpolation of data sets (y)")
figure(3)
title("load scaling using interpolation of data sets (x)")