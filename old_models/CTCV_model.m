function zdot = CTCV_model(t,z,u,d,th)

% coordinated turn motion model state space representation

zdot(1,1) = z(3,1)*cos(z(4,1));
zdot(2,1) = z(3,1)*sin(z(4,1));
zdot(3,1) = d(1,1);
zdot(4,1) = z(5,1);
zdot(5,1) = d(2,1);

end