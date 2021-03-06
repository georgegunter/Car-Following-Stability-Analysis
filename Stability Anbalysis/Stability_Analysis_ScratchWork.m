%% String Stability:

p = [20,.5,20.72,2.23];

s_vals = 1:.1:20;

lambda_vals = string_stability(p,s_vals);

figure()
hold on
plot(s_vals,lambda_vals,'LineWidth',2)
plot(s_vals,zeros(size(s_vals)),'k--','LineWidth',.5)
plot(4.86,string_stability(p,4.86),'r*','MarkerSize',15)
%% Look at V(s):
b=p(1);a=p(2);vm=p(3);d0=p(4);
V = @(s) vm*(tanh(s./d0-2)+tanh(2))/(1+tanh(2));
V_prime = @(s) vm/(1+tanh(2))*(1-tanh(s/d0-2).^2);

s_vals = 0:.1:30;

figure()
subplot(2,1,1)
plot(s_vals,V(s_vals))
subplot(2,1,2)
plot(s_vals,V_prime(s_vals))

%% Look at affect of the parameter a:
s_vals = 0:.1:30;

a_vals = [5,10,15,20,25,30];

figure()
hold on

for i=1:length(a_vals)   

    p = [a_vals(i),.5,10,4.5,0,0];

    lambda_vals = string_stability(p,s_vals);

    plot(s_vals,lambda_vals,'LineWidth',2)
end

plot(s_vals,zeros(size(s_vals)),'k--','LineWidth',.5)

legend('5','10','15','20','25','30')
title('Varying \alpha')


%% Look at affect of the parameter b:
s_vals = 0:.1:30;

b_vals = [.2,.4,.5,.7,1,1.5];

figure()
hold on

for i=1:length(b_vals)   

    p = [20,b_vals(i),10,4.5,0,0];

    lambda_vals = string_stability(p,s_vals);

    plot(s_vals,lambda_vals,'LineWidth',2)
end

plot(s_vals,zeros(size(s_vals)),'k--','LineWidth',.5)

legend('.2','.4','.5','.7','1','1.5')
title('Varying \beta')

%% Look at affect of the parameter s0:
s_vals = 0:.1:30;

s0_vals = [1,4.5,5,10,15,20];

legend_string = cell(length(s0_vals),1);

figure()
hold on

for i=1:length(s0_vals)   

    p = [20,.5,10,s0_vals(i),0,0];

    lambda_vals = string_stability(p,s_vals);

    plot(s_vals,lambda_vals,'LineWidth',2)
    
    legend_string{i} = num2str(s0_vals(i));
end

plot(s_vals,zeros(size(s_vals)),'k--','LineWidth',.5)

legend(legend_string)
title('Varying S_{0}')

%% Look at affect of the parameter vm:
s_vals = 0:.1:30;

vm_vals = [5,7.5,10,15,20,30];

legend_string = cell(length(vm_vals),1);

figure()
hold on

for i=1:length(vm_vals)   

    p = [20,.5,vm_vals(i),4.5,0,0];

    lambda_vals = string_stability(p,s_vals);

    plot(s_vals,lambda_vals,'LineWidth',2)
    
    legend_string{i} = num2str(vm_vals(i));
end

plot(s_vals,zeros(size(s_vals)),'k--','LineWidth',.5)

legend(legend_string)
title('Varying V_{m}')



%% RDC:
s_vals = 0:.1:100;

b=p(1);a=p(2);vm=p(3);d0=p(4);

V_prime = @(s) vm/(1+tanh(2))*(1-tanh(s/d0-2)).^2;

f_s = -a*V_prime(s_vals);
f_v = -a;
f_dV = s_vals*(1/b);

figure()
plot(s_vals,f_s,'LineWidth',2)







