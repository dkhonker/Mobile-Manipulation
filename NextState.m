function nextState = NextState2(current_conf, control, dt, max_speed)
% current_conf -> q = [Φ,x,y,j1,j2,j3,j4,j5,θ1,θ2,θ3]
% control -> u = [j1',j2',j3',j4',j5',θ1',θ2',θ3']
% dt -> 时间间隔
% max_speed -> 最大速度, default : None
tmp = control([5,6,7,8,9,1,2,3,4]);
control = tmp;
if nargin == 4
    control(control > max_speed) = max_speed;
    control(control < -1*max_speed) = -1*max_speed;
end

nextState(4:8) = current_conf(4:8) + control(1:5) * dt;
nextState(9:12) = current_conf(9:12) + control(6:9) * dt;
r = 0.0475;
w= 0.3/2;
l = 0.47/2;
F =  r/4 .* [-1/(l+w),1/(l+w),1/(l+w),-1/(l+w);1,1,1,1;-1,1,-1,1];
Vb = F * control(6:9).' * dt;
if(Vb(1)==0)
    tmp=Vb;
    %tmp(1)=0;
    dqb = tmp;
else
    dqb = [Vb(1); ((Vb(2)*sin(Vb(1))+Vb(3)*(cos(Vb(1))-1))/Vb(1)); ((Vb(3)*sin(Vb(1))-Vb(2)*(cos(Vb(1))-1))/Vb(1))];
end
faik = current_conf(1);
dq = [1,0,0;0,cos(faik),-sin(faik);0,sin(faik),cos(faik)] * dqb;
nextState(1:3) = current_conf(1:3) + dq.';
end
