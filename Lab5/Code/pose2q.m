function q = pose2q(x,y,z,phi,psi)
theta = atan2(y,x);
s1 = sin(theta); s2 = sin(phi); s3 = sin(psi);
c1 = cos(theta); c2 = cos(phi); c3 = cos(psi);
R = [-c1*s2*c3+s1*s3, c1*s2*s3+s1*c3, c1*c2;
     -c1*s3-s1*s2*c3, s1*s2*s3-c1*c3, c2*s1;
     c2*c3, -c2*s3, s2];
pos = [x y z]';
T = vertcat(horzcat(R,pos),[0,0,0,1]);
qsol = IK_lynx_sol(T);
q = horzcat(qsol(2,:),0);
end

