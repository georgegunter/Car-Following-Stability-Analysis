function [amp_factor] = Bando_Transfer_Function_Val(p,s,omega)

b=p(1);a=p(2);vm=p(3);d0=p(4);

V_prime = @(d) vm/(1+tanh(2))*(1-tanh(d/d0-2).^2);

f_s = a*V_prime(s);
f_v = -a;
f_dV = b*(s.^-2);
f_1 = f_v;f_2=f_s;f_3=f_dV;
% Comes from Monteil et. al. 2018
amp_factor = sqrt(((omega.^2).*f_3^2+f_2.^2)./...
        ((f_2 - omega.^2).^2 + (omega.^2)*(f_3-f_1).^2));

end

