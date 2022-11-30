function result_traj =TrajectoryGenerator(Tseinitial, Tscinitial, Tscfinal, Tcegrasp, Tcestandoff, k)
%result_traj=cell(8,k*100);
N=300;
result_traj1=ScrewTrajectory(Tseinitial, Tscinitial*Tcestandoff, 3, N, 5);
for i=1:N
    m=result_traj1{i};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 0];
    if i==1
        result_traj=n;
    else
        result_traj = [result_traj;n];
    end
end
N=100;
result_traj2=ScrewTrajectory(Tscinitial*Tcestandoff, Tscinitial*Tcegrasp, 3, N, 5);
for i=1:N
    m=result_traj2{i};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 0];
    result_traj = [result_traj;n];
end
N=100;
result_traj3=ScrewTrajectory(Tscinitial*Tcegrasp, Tscinitial*Tcegrasp, 3, N, 5);
for i=1:k*100
    m=result_traj3{i};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 1];
    result_traj = [result_traj;n]
end
N=100;
result_traj4=ScrewTrajectory(Tscinitial*Tcegrasp, Tscinitial*Tcestandoff, 3, N, 5);
for i=1:N
    m=result_traj4{i};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 1];
    result_traj = [result_traj;n];
end 
N=500;
result_traj5=ScrewTrajectory(Tscinitial*Tcestandoff, Tscfinal*Tcestandoff, 3, N, 5);
for i=1:N
    m=result_traj5{i};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 1];
    result_traj = [result_traj;n];
end  
N=100;
result_traj6=ScrewTrajectory(Tscfinal*Tcestandoff, Tscfinal*Tcegrasp, 3, N, 5);
for i=1:N
    m=result_traj6{i};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 1];
    result_traj = [result_traj;n];
end 
N=100;
result_traj7=ScrewTrajectory(Tscfinal*Tcegrasp, Tscfinal*Tcegrasp, 3, N, 5);
for i=1:N
    m=result_traj7{i};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 0];
    result_traj = [result_traj;n];
end  
result_traj8=ScrewTrajectory(Tscfinal*Tcegrasp, Tscfinal*Tcestandoff, 3, 100, 5);
for i=1:N
    m=result_traj8{i};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 0];
    result_traj = [result_traj;n];
end  
end
