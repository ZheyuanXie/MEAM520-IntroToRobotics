lynxStart()

traj_type = 'circular'; % linear or circular
video_out = false;

%% Trajectory Generation
d1 = 76.2;

if strcmp(traj_type,'linear')
    t = 0:0.05:4;
    x = 300*ones(size(t));
    dx = 0*ones(size(t));
    y = -150+75*t;
    dy = 150*ones(size(t));
    z = -100+100*t;
    dz = 200*ones(size(t));
elseif strcmp(traj_type,'circular')
    t = 0:0.05:4;
    x = 300*ones(size(t));
    dx = 0*ones(size(t));
    y = 100*sin(2*t);
    dy = 200*cos(2*t);
    z = 100*cos(2*t)+200;
    dz = -200*sin(2*t);
else
    fprintf('Invalid traj_type\n');
    return
end

plot3(x,y,z,'LineWidth',2,'Color','r');

% Orientation
theta = atan2(y,x);
dtheta = (-y.*dx + x.*dy)./(x.^2+y.^2);
phi = zeros(size(t));%pi/8*sin(2*t);%atan2(z-d1,sqrt(x^2+y^2));
dphi = zeros(size(t));%pi/8*2*cos(2*t);
psi = zeros(size(t));%5*t;
dpsi = zeros(size(t));%5*ones(size(t));


%% IK & VIK
qlog = zeros(5,numel(t));
qdotlog = zeros(5,numel(t));
if video_out
    video = VideoWriter('sim.avi');
    video.FrameRate = 20;
    open(video)
end
for i = 1:numel(t)
    s1 = sin(theta(i)); s2 = sin(phi(i)); s3 = sin(psi(i));
    c1 = cos(theta(i)); c2 = cos(phi(i)); c3 = cos(psi(i));
    R = [-c1*s2*c3+s1*s3, c1*s2*s3+s1*c3, c1*c2;
         -c1*s3-s1*s2*c3, s1*s2*s3-c1*c3, c2*s1;
         c2*c3, -c2*s3, s2];
    pos = [x(i) y(i) z(i)];
    T = vertcat(horzcat(R,pos'),[0,0,0,1]);
    qsol = IK_lynx_sol(T);
    q = horzcat(qsol(2,:),0);
    qlog(:,i)=qsol(2,:)';
    
    wx = sin(theta(i))*dphi(i)+cos(theta(i))*cos(phi(i))*dpsi(i);
    wy = -cos(theta(i))*dphi(i)+sin(theta(i))*cos(phi(i))*dpsi(i);
    wz = dtheta(i) + sin(phi(i))*dpsi(i);
    
    e_vel=[dx(i);dy(i);dz(i);wx;wy;wz];
    qdot = IK_velocity(qsol(2,:),e_vel);
    qdotlog(:,i)=qdot(1,1:5);
    
    plotLynx(q)
    if video_out
        frame = getframe(gcf);
        writeVideo(video,frame);
    end
end
if video_out
    close(video);
end

%% Plot Result
subplot(2,1,1)
plot(t,qlog,'LineWidth',2)
legend('q1','q2','q3','q4','q5')
xlabel('Time (s)')
ylabel('Joint Angles (rad)')

subplot(2,1,2) 
plot(t,qdotlog,'LineWidth',2)
legend('dq1','dq2','dq3','dq4','dq5')
xlabel('Time (s)')
ylabel('Joint Angular Velocity (rad/s)')
