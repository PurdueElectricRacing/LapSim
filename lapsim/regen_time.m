function [bat_volt, bat_curr, t] = regen_time(ms, mt, R, C, OCV, SOC, cell_cap, mo, bat_cap, cap_voltage, DCDC_voltage)
if SOC > 95
    x = SOC - 95;
    curr_limit = (10 * cell_cap) - ((10 * cell_cap) * exp(5 - x));
else
    curr_limit = 10 * cell_cap;
end
    [t,x] = ode15s(@(t,x) regen_ode(t,x,R,C,OCV, -curr_limit, l.lv_power_cons), [0, 0.01], [bat_cap, cap_voltage, DCDC_voltage]);
    bat_volt = OCV + (curr_limit * R(1)) - x(:,1);
    Itotal = curr_limit + ((bat_volt - x(:,2)) / R(4));
    m_voltage = bat_volt + (Itotal * R(3));
    m_power = mean(m_voltage .* Itotal);
    power_vector = interp2(mo.motor_speed_table, mo.motor_trq_table, mo.motor_eff_table_discharge, ms, mo.motor_trq_table(:,1));
    power_vector = power_vector .* ((ms * mo.motor_trq_table(:,1) * 2 * pi()) / 60);
    power_vector = power_vector / 25;
    if(m_power > power_vector(end:end))
        %run HV_ODE
    end
    regen_torque = interp1(power_vector, mo.motor_trq_table(:,1), m_power);
    if regen_torque > -mt
        %run HV_ODE
    end
    batt_curr = -curr_limit;
end