function Xref = spiralTrajectory(T, Ts, radius, height, turns, p0, q0)

    t = 0:Ts:T; % Vector of times
    N = numel(t);
    tau = t/T; % Normalization of time

    % arc length
    s = 10*tau.^3-15*tau.^4+6*tau.^5;
    s_dot = (30*tau.^2-60*tau.^3+30*tau.^4)/T;

    % angle reference
    theta = 2*pi*turns*s;
    theta_dot = 2*pi*turns*s_dot;

    % position reference ...
    x_local = radius*cos(theta)-radius;
    y_local = radius*sin(theta);
    z_local = height*s;

    % ... respect to the initial position
    x = p0(1) + x_local;
    y = p0(2) + y_local;
    z = p0(3) + z_local;
    pos = [x; y; z];

    % linear velocity reference
    vx_local = -radius*sin(theta).*theta_dot;
    vy_local = radius*cos(theta).*theta_dot;
    vz_local = height*s_dot;
    vel = [vx_local; vy_local; vz_local];

    % yaw reference ...
    yaw_local = theta + pi/2;

    % ... respect to inital orientation
    eul0 = quat2eul(q0.', "ZYX");
    yaw0 = eul0(1);
    delta = yaw0 - yaw_local(1);
    yaw   = yaw_local + delta;

    % pitch and roll references
    pitch = zeros(1, N);
    roll  = zeros(1, N);

    % quaternion construction
    cy = cos(yaw/2);   sy = sin(yaw/2);
    cp = cos(pitch/2); sp = sin(pitch/2);
    cr = cos(roll/2);  sr = sin(roll/2);
    qw = cr.*cp.*cy + sr.*sp.*sy;
    qx = sr.*cp.*cy - cr.*sp.*sy;
    qy = cr.*sp.*cy + sr.*cp.*sy;
    qz = cr.*cp.*sy - sr.*sp.*cy;
    q = [qw; qx; qy; qz];

    % angular velocity reference
    yaw_dot = theta_dot;
    angvel  = [zeros(1,N); zeros(1,N); yaw_dot];

    % Final trajectory
    Xref = [pos; q; vel; angvel];
end
