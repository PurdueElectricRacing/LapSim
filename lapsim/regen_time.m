function [bat_volt, bat_curr, mt, t] = regen_time(DCDCpd, ms, mt, R, C, OCV, SOC, cell_cap, mo, bat_cap, DCDC_voltage)
if SOC > 95
    x = SOC - 95;
    curr_limit = (10 * cell_cap) - ((10 * cell_cap) * exp(x - 5));
else
    curr_limit = 10 * cell_cap;
end
    [t,x] = ode15s(@(t,x) regen_ode(t,x,R,C,OCV, -curr_limit, DCDCpd), [0, 0.01], [bat_cap, DCDC_voltage]);
    bat_volt = OCV + (curr_limit * R(1)) - x(:,1);
    bat_curr = repmat(-curr_limit, size(bat_volt, 1), 1);
    Itotal = curr_limit + ((bat_volt - x(:,2)) / R(4));
    m_voltage = bat_volt + (Itotal * R(3));
    m_power = mean(m_voltage .* Itotal);
    m_power = m_power / 4;
    power_vector = interp2(mo.motor_speed_table, mo.motor_trq_table, mo.motor_eff_table_discharge, ms, mo.motor_trq_table(:,1));
    power_vector = power_vector .* ((ms * mo.motor_trq_table(:,1) * 2 * pi()) / 60);
    power_vector = power_vector / 100;
    skew = linspace(0,size(power_vector,1) * 0.1, size(power_vector,1));
    power_vector = power_vector + skew.';
    if(m_power > power_vector(end:end))
        %run HV_ODE
        mt = min([30, (15000 * 60 / (2 * pi() * ms)), abs(mt)]);
        mt = -mt;
        bat_volt = NaN;
        bat_curr = NaN;
        return
    end
    regen_torque = interp1(power_vector, mo.motor_trq_table(:,1), m_power);
    if regen_torque > abs(mt)
        %run HV_ODE
        bat_volt = NaN;
        bat_curr = NaN;
        return
    end
end