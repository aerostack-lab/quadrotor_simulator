function [A, B, C, D, G_Y_ss, G_Y, A_dY, B_dY, C_dY, D_dY, G_dY, G_dY_ss] = create_yaw_model( DYAW_SCALE, Tp, Zeta)
% Yaw:
% ----
% Input:  dYaw/dt command [counts]
% Output: [Yaw; dYaw/dt]; [rad; rad/s]
% dYc >> second-order system >> dY >> integrator >> Y
%
% Example of input to funtion:
% ----------------------------
% DYAW_SCALE = 2047 / 254.760; % counts/deg/s, rc/controller input
% Tp         = 0.5;            % seconds , desired peak time
% Zeta       = 0.80;           % 0<Zeta<1, adimensional, desired damping coeffcient
%
% Model in equations:
% -------------------
% In both cases:
%    d(x)/dt = A*x + B*u
%          y = C*x + D*u
%
% [*] G_Y_ss:
%
%     | Yaw           |
%     |               |                      | Yaw     |
% x = | dYaw/dt       |; u = (dYaw/dt)c; y = |         |
%     |               |                      | dYaw/dt |
%     | d^2(Yaw)/dt^2 |
%
% [*] G_dY_ss:
%
%     | Yaw     |
% x = |         |; u = (dYaw/dt)c; y = dYaw/dt
%     | dYaw/dt |

wn = 3/(Tp*sqrt(1-Zeta^2));
Kp = (pi/180)/DYAW_SCALE;  % (rad/s)/count

try
    s    = tf('s');
    G_dY = Kp*wn^2/(wn^2+2*Zeta*wn*s+s^2);
    G_Y  = G_dY*(1/s);
catch
    G_dY = 0.0;
    G_Y  = 0.0;
end

A_dY = [   0          1;
    -wn^2 -2*Zeta*wn];
B_dY = [      0;
    Kp*wn^2];
C_dY = [1  0];
D_dY = 0;

A = [0     1          0;
    0     0          1;
    0 -wn^2 -2*Zeta*wn];
B = [      0;
    0;
    Kp*wn^2];
C = [1 0 0;
    0 1 0];
D = [0;
    0];

try
    G_dY_ss = ss(A_dY, B_dY, C_dY, D_dY);
    G_Y_ss  = ss(A, B, C, D);
catch
    G_dY_ss = 0.0;
    G_Y_ss  = 0.0;
end


% % Example test code:
% % ------------------
% [dY0 T0_dY0g] = step((180/pi)*G_dY);
% [Y0  T0_Y0g ] = step((180/pi)*G_Y, 1);
% [Yallss T0ss] = step((180/pi)*G_Y_ss, 1);
% Y0ss  = Yallss(:,1);
% dY0ss = Yallss(:,2);
% % plot dY
% figure()
% plot(T0_dY0g,dY0)
% hold all
% plot(T0ss,dY0ss)
% hold off
% % plot Y
% figure()
% plot(T0_Y0g,Y0)
% hold all
% plot(T0ss,Y0ss)
% hold off

end


