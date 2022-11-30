clc;
Tseinitial=[[0 0 1 0];[0 1 0 0];[-1 0 0 0.5];[0 0 0 1]];
Tscinitial=[[1 0 0 1];[0 1 0 0];[0 0 1 0.025];[0 0 0 1]];
Tscfinal=[[0 1 0 0];[-1 0 0 -1];[0 0 1 0.025];[0 0 0 1]];
Tcegrasp=[[cos(3*pi/4) 0 sin(3*pi/4) 0];[0 1 0 0];[-sin(3*pi/4) 0 cos(3*pi/4) -0];[0 0 0 1]];
Tcestandoff=[[cos(3*pi/4) 0 sin(3*pi/4) 0.];[0 1 0 0];[-sin(3*pi/4) 0 cos(3*pi/4) 0.1];[0 0 0 1]];
result_traj =TrajectoryGenerator(Tseinitial, Tscinitial, Tscfinal, Tcegrasp, Tcestandoff, 1);
%result_traj=reshape(result_traj',[800,1]);
csvwrite('q_k_csv_file.csv',result_traj);
result_traj = csvread("q_k_csv_file.csv");
%result_traj = csvread("end_eff.csv");
N = length(result_traj);

Xd = [result_traj(1,1) result_traj(1,2) result_traj(1,3) result_traj(1,10);
result_traj(1,4) result_traj(1,5) result_traj(1,6) result_traj(1,11);
result_traj(1,7) result_traj(1,8) result_traj(1,9) result_traj(1,12);
 0 0 0 1];
%Xdtmp=Xd;

Xdnext = [result_traj(2,1) result_traj(2,2) result_traj(2,3) result_traj(2,10);
result_traj(2,4) result_traj(2,5) result_traj(2,6) result_traj(2,11);
result_traj(2,7) result_traj(2,8) result_traj(2,9) result_traj(2,12);
0 0 0 1];

current_conf = [0.1 0.1 0.2 0 0 0.2 -1.6 0 0 0 0 0];

Tb0 = [ 1 0 0 0.1662;
 0 1 0 0;
 0 0 1 0.0026;
 0 0 0 1];

Blist = [[0; 0; 1; 0; 0.033; 0],...
 [0; -1; 0; -0.5076; 0; 0],...
 [0; -1; 0; -0.3526; 0; 0],...
 [0; -1; 0; -0.2176; 0; 0],...
 [0; 0; 1; 0; 0; 0]];


M0e = [ 1 0 0 0.033;
 0 1 0 0;
 0 0 1 0.6546;
 0 0 0 1];


result_conf = zeros(N-1, 13);
result_Xerr = zeros(N-1, 6);
Kp=eye(6)*50;
Ki=eye(6)*15;
dt=0.01;
for i=1:N-2
    thetalist(:,1) = current_conf(4:8);
    Xd = [result_traj(i,1) result_traj(i,2) result_traj(i,3) result_traj(i,10);
 result_traj(i,4) result_traj(i,5) result_traj(i,6) result_traj(i,11);
 result_traj(i,7) result_traj(i,8) result_traj(i,9) result_traj(i,12);
 0 0 0 1];
    Xdnext =[result_traj(i+1,1) result_traj(i+1,2) result_traj(i+1,3) result_traj(i+1,10);
result_traj(i+1,4) result_traj(i+1,5) result_traj(i+1,6) result_traj(i+1,11);
result_traj(i+1,7) result_traj(i+1,8) result_traj(i+1,9) result_traj(i+1,12);
0 0 0 1];

    phi = current_conf(1);
    x = current_conf(2);
    y = current_conf(3);
    thetalist(:,1) = current_conf(4:8);
    
    
    Tsb_phi = [ cos(phi) -sin(phi) 0 x;
        sin(phi) cos(phi) 0 y;
        0 0 1 0.0963;
        0 0 0 1];
    T0e = FKinBody(M0e, Blist, thetalist);
    X = Tsb_phi*Tb0*T0e;
    [control, Xerr] = FeedbackControl(X, Xd, Xdnext, Kp, Ki, dt, thetalist);
    [current_conf] = NextState(current_conf, control.', dt);
    result_conf(i,1:12) = current_conf;
    result_conf(i,13) = result_traj(i,13);
    result_Xerr(i,:) = Xerr;
end
csvwrite('result_conf.csv',result_conf);
csvwrite('result_Xerr.csv',result_Xerr);
