s_eq = 4;
Params = [20,.5,9.72,2.23];

omega_vals = [.0001,.001,.01,.1,1,10];

amp_vals = zeros(size(omega_vals));

for i=1:length(omega_vals)
    amp_vals(i) = find_Amp_Factor_numerical(Params,s_eq,omega_vals(i));
end

plot(log10(omega_vals),amp_vals)