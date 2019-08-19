function [dV,dS,dP] = Bando_FTL_RK_Step(RingRoad_Params,Bando_Params,v,v_l,s,p)

dt = RingRoad_Params.dt;
n = RingRoad_Params.number_cars;

dV = zeros(n,1);
dS = zeros(n,1);
dP = zeros(n,1);

for i=1:n
    k1 = [Bando_FTL_Accel(Bando_Params,v(i),v_l(i),s(i)),v_l(i)-v(i),v(i)]*dt;
    k2 = [Bando_FTL_Accel(Bando_Params,v(i)+k1(1)/2,v_l(i),s(i)+k1(2)/2),v_l(i)-v(1)+k1(1)/2,v(i)+k1(3)/2]*dt;
    k3 = [Bando_FTL_Accel(Bando_Params,v(i)+k2(1)/2,v_l(i),s(i)+k2(2)/2),v_l(i)-v(i)+k2(1)/2,v(i)+k2(3)/2]*dt;
    k4 = [Bando_FTL_Accel(Bando_Params,v(i)+k3(1),v_l(i),s(i)+k3(2)),v_l(i)-v(i)+k3(1),v(i)+k3(3)]*dt;

    update_step = k1/6 + k2/3 + k3/3 + k4/6;
    dV(i) = update_step(1);
    dS(i) = update_step(2);
    dP(i) = update_step(3);
    
end

end

