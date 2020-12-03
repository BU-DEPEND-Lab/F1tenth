function SPEC(barrel_size, dis_barrel, dis_wall, dis_target, visual)
[g, data, dataTraj, tau2, dCar, empty_flag] = Grid_data(barrel_size, dis_barrel, dis_wall, dis_target, visual);
subcar = rossubscriber('Car_Pose');
substart = rossubscriber('intersect_flag');
start = receive(substart);
pubpara = rospublisher('drive_parameters');
pubflag = rospublisher('empty_flag');
msg = rosmessage(pubpara);
location = receive(subcar);
xinit = [location.X, location.Y, location.Theta];
[traj, traj_u, traj_tau, failure] = compute_traj(xinit, g, data, tau2, visual)
flag = empty_flag
send(pubflag,flag)
while empty_flag && start
    location = receive(subcar);
    x = [location.X, location.Y, location.Theta];
    [traj_u, failure] = compute_ctrl(x, g, data, dataTraj, tau2, dCar, visual);
    msg.Velocity = 10;
    msg.Angle = traj_u;
    send(pubpara,msg);
end 
end
