Fx = [];
PC = [];
for l = 4.448*[50,100,150,200,250]
    PC = [PC; magic_formula_corner(l, 0, 1)];
    for s = -0.3:0.01:0.3
        Fx = [Fx,calc_magic_formula(PC(end,:),s)];
    end
    plot((-0.3:0.01:0.3), Fx,"m");
    hold on;
    Fx = [];
end