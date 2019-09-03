function [amp_factors] = Bando_Bode_Plot(p,s)

omega_vals = -10:.1:1;
omega_vals = 10.^omega_vals;
amp_factors = Bando_Transfer_Function_Val(p,s,omega_vals);
log_omega_vals = log10(omega_vals);

figure()
hold on
plot(log_omega_vals,amp_factors,'LineWidth',2)
plot([log_omega_vals(1) log_omega_vals(end)],[1,1],'k-','LineWidth',1)

end

