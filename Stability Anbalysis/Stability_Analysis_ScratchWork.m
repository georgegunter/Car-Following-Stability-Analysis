
p = [20,.5,10,2.5,0,0];



delta_V_vals = -10:.1:10;
s_vals = 2:.1:20;
%% String Stability:

p = [20,.5,10,4.5,0,0];

s_vals = 0:.1:30;

lambda_vals = string_stability(p,10,10,s_vals);

% figure()
plot(s_vals,lambda_vals)

%% RDC:
s_vals = 0:.1:100;

f_s_1 = @(s) (a*vm/(d0*(1+tanh(2))))*(1-tanh(s/d0-2).^2);

f_s_2 = @(s) ...
    ((Bando_FTL_Accel(p,10,10,s_vals+.01)-(Bando_FTL_Accel(p,10,10,s_vals)))./.01);

f_s_2_Vals = f_s_2(s_vals);
f_s_1_Vals = f_s_1(s_vals);

figure()
% plot(s_vals,f_s_2_Vals,'LineWidth',2)
plot(s_vals,f_s_1_Vals,'LineWidth',2)







