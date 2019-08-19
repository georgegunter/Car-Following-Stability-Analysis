function [accel] = Bando_FTL_Accel(Params,v,v_l,s)

b = Params(1);
a = Params(2);
vm = Params(3);
d0 = Params(4);

% V = @(s) (tanh(s./x-y*(d0)) + 1)*(vm/2);

V = @(d) vm*(tanh(d./d0-2)+tanh(2))/(1+tanh(2));

accel = a*(V(s)-v) + b*((v_l - v)./(s.^2));


end

