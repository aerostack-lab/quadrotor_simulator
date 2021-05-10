function [A, B, C, D, G_z_ss] = create_z_model( THRUST_SCALE, Tth, m, vzmax, pitch_sym, roll_sym)
% z:
% ----
% Input:        thrust command   [counts]
% Second input: mass_real / mass_est ~= 1
% Output: [ z; dz/dt]            [m; m/s]
% thrust (and P and R) >> second-order system >> dz >> integrator >> z 
%
% Example of input to funtion:
% ----------------------------
% THRUST_SCALE = 4095 / 32; % counts/N - approximate, rc/controller input
% Tth   = 0.15/3;           % sec, response time of propellers
% m     = 1.5;              % kg, estimated mass of the UAV
% vzmax = 2.5;              % m/s
% pitch_sym = 0; roll_sym = 0;       % not a symbolic model
% pitch_sym = P; roll_sym  = syms R; % symbolic model
%
% Model in equations:
% -------------------
%    d(x)/dt = A*x + B*u
%          y = C*x + D*u
%
% [*] G_z_ss:
%
%     | thrust |                       | z      |
%     |        |      | thrustc |      |        |
% x = | z      |; u = |         |; y = | dz/dt  |
%     |        |      | m_real  |      |        |
%     | dz/dt  |                       | thrust |

% g     = 9.81;                 % m/s^2
syms g real;
C     = (32 - m*g)/(2*vzmax); % N/(m/s)^2, C = (T - mg)/2*vzmax
Kp    = cos(pitch_sym)*cos(roll_sym)/(THRUST_SCALE);         % N/count

A = [-1/Tth      0          1;
          0      0          1; 
       Kp/m      0     -2*C/m];
B = [  -1/Tth   0;
            0   0; 
            0  +g];
% C = [0  1  0;   % z
%      0  0  1;   % dz/dt
%      1  0  0];  % thrust
% D = [0 0; 0 0; 0 0];
C = [0  1  0 ;   % z
     0  0  1 ];  % dz/dt
D = [0 0; 0 0];

try
    g     = 9.81;
    A = eval(A);
    B = eval(B);
    G_z_ss = ss(A, B, C, D);
catch
    syms g real;
    G_z_ss = 0.0;
end

% % Example test code:
% % ------------------
% thrust = 15* (4095/32); % "x" [N] * gain
% [Zallss T0ss] = step(G_Z_ss, 1);
% Z_T  = Zallss(:,1,1)*thrust;
% dZ_T = Zallss(:,2,1)*thrust;
% Z_p  = Zallss(:,1,2);
% dZ_p = Zallss(:,2,2);
% % plot Z
% figure
% plot(T0ss,Z_T+Z_p)
% % plot dZ
% figure
% plot(T0ss,dZ_T+dZ_p)

end


