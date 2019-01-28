function path = Trajectory(n)
load('trajectoryCoordinates.mat')
path(1) = Traj(n,2);
path(2) = Traj(n,3);
path(3) = Traj(n,4);
path(4) = Traj(n,5);
end