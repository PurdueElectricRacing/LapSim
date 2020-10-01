bat.SOC_current = 100;
clear SOC bat_volt bat_curr time trqe


bat_ah = bat.total_cap / (bat.cell_s * mean(bat.OCV_table(:,2)));
R(1) = interp1(bat.cell_r1_table(:,1),bat.cell_r1_table(:,2),bat.SOC_current, 'makima');
R(2) = interp1(bat.cell_r2_table(:,1),bat.cell_r2_table(:,2),bat.SOC_current, 'makima');
R = R / 1000;
R = R * (bat.cell_s / bat.cell_p);
C = interp1(bat.cell_c1_table(:,1),bat.cell_c1_table(:,2),bat.SOC_current, 'makima');
C = C * (bat.cell_p / bat.cell_s);
OCV = interp1(bat.OCV_table(:,1),bat.OCV_table(:,2),bat.SOC_current);
R(3) = h.cable_r;
R(4) = h.cable_r;
C(2) = mo.DC_link_cap;
C(3) = h.DCDC_input_cap;
OCV = OCV * bat.cell_s;
cap_voltage = OCV;
bat_cap = 0.001;
DCDC_voltage = OCV;
endj = 0;
for k = 1:26
    for i = 0:0.01:tme(end)
        j = cast((i * 100)+ 1, 'int64');
        tstart = i + (tme(end) * (k - 1));
        mpd = pd(j);
        mpd = mpd * 4;
        DCDCpd = pdlv(j);
        if mpd == -4
            [bat.voltage_out, bat.current_out, mt(j), t] = regen_time(DCDCpd, ms(j), mt(j), R, C, OCV, bat.SOC_current, bat_ah, mo, bat_cap, DCDC_voltage);
            if isnan(bat.voltage_out)
%                 disp("dingus")
                mpd = 4 * ((mt(j) * ms(j) * 2 * pi()) / 60);
%                 disp(mpd)
                [t,x] = ode15s(@(t,x) HV_ODE(t,x,OCV,R,C,mpd, DCDCpd), [tstart, tstart + 0.01], [bat_cap, cap_voltage, DCDC_voltage]);
                bat.voltage_out = (OCV+((R(1)/R(3)).*x(:,2)) +((R(1)/R(4)).*x(:,3)) - x(:,1))/(1+(R(1)/R(3))+(R(1)/R(4)));
                bat.current_out = ((bat.voltage_out - x(:,2))/R(3)) + ((bat.voltage_out - x(:,3))/R(4));
            end
                
        else
            [t,x] = ode15s(@(t,x) HV_ODE(t,x,OCV,R,C,mpd, DCDCpd), [tstart, tstart + 0.01], [bat_cap, cap_voltage, DCDC_voltage]);
            bat.voltage_out = (OCV+((R(1)/R(3)).*x(:,2)) +((R(1)/R(4)).*x(:,3)) - x(:,1))/(1+(R(1)/R(3))+(R(1)/R(4)));
            bat.current_out = ((bat.voltage_out - x(:,2))/R(3)) + ((bat.voltage_out - x(:,3))/R(4));
        end
        bat.SOC_current = bat.SOC_current - 100 * (trapz(t/3600, bat.current_out) / bat_ah);
        R(1) = interp1(bat.cell_r1_table(:,1),bat.cell_r1_table(:,2),bat.SOC_current, 'makima') / 1000;
        R(1) = R(1) * (bat.cell_s / bat.cell_p);
        R(2) = interp1(bat.cell_r2_table(:,1),bat.cell_r2_table(:,2),bat.SOC_current, 'makima') / 1000;
        R(2) = R(2) * (bat.cell_s / bat.cell_p);
        C(1) = interp1(bat.cell_c1_table(:,1),bat.cell_c1_table(:,2),bat.SOC_current, 'makima');
        C(1) = C(1) * (bat.cell_p / bat.cell_s);
        OCV = interp1(bat.OCV_table(:,1),bat.OCV_table(:,2),bat.SOC_current) * bat.cell_s;
        bat_cap = x(end,1);
        cap_voltage = x(end,2);
        DCDC_voltage = x(end,3);
        SOC(j + endj) = bat.SOC_current;
        bat_volt(j + endj) = bat.voltage_out(end);
        bat_curr(j + endj) = bat.current_out(end);
        time(j + endj) = tstart;
        trqe(j + endj) = mt(j);
        if(bat_curr(end) < -120)
            disp("over regen")
            return
        end
        if SOC(end) < 0
            disp('Batteries dead')
            return
        end
    end
    info = sprintf('Lap %d complete. SOC = %.2f', k, bat.SOC_current); 
    disp(info)
    endj = endj + j;
end