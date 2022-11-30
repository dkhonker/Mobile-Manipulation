function result_traj =TrajectoryGenerator(Tseinitial, Tscinitial, Tscfinal, Tcegrasp, Tcestandoff, k)
result_traj=cell(8,k*100);
result_traj1=ScrewTrajectory(Tseinitial, Tscinitial*Tcestandoff, 1, k*100, 3);
for i=1:k*100
    m=result_traj1{i};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 0];
    result_traj{1,i}=n;
end
result_traj2=ScrewTrajectory(Tscinitial*Tcestandoff, Tscinitial*Tcegrasp, 1, k*100, 3);
for i=1:k*100
    m=result_traj2{i};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 0];
    result_traj{2,i}=n;
end
for i=1:k*100
    m=result_traj2{k*100};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 1];
    result_traj{3,i}=n;
end
result_traj4=ScrewTrajectory(Tscinitial*Tcegrasp, Tscinitial*Tcestandoff, 1, k*100, 3);
for i=1:k*100
    m=result_traj4{i};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 1];
    result_traj{4,i}=n;
end 
result_traj5=ScrewTrajectory(Tscinitial*Tcestandoff, Tscfinal*Tcestandoff, 1, k*100, 3);
for i=1:k*100
    m=result_traj5{i};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 1];
    result_traj{5,i}=n;
end  
result_traj6=ScrewTrajectory(Tscfinal*Tcestandoff, Tscfinal*Tcegrasp, 1, k*100, 3);
for i=1:k*100
    m=result_traj6{i};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 1];
    result_traj{6,i}=n;
end 

for i=1:k*100
    m=result_traj6{k*100};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 0];
    result_traj{7,i}=n;
end  
result_traj8=ScrewTrajectory(Tscfinal*Tcegrasp, Tscfinal*Tcestandoff, 1, k*100, 3);
for i=1:k*100
    m=result_traj8{i};
    n = m(1:3,1:3)';
    n=reshape(n,1,9);
    n=[n m(1,4) m(2,4) m(3,4)];
    n=[n 0];
    result_traj{8,i}=n;
end  
end
