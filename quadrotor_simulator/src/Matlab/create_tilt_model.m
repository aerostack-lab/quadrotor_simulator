function [A, B, C, D, G_tilt_ss, G_tilt] = create_tilt_model( TILT_SCALE, Tp)
% Tilt:
% ----
% Input:  tilt command [counts]
% Output: tilt         [rad]
% tiltc >> first-order system >> tilt
%
% Example of input to funtion:
% ----------------------------
% TILT_SCALE = 40;   % counts/deg, rc/controller input
% Tp         = 0.10; % seconds , desired response time
%
% Model in equations:
% -------------------
%    d(x)/dt = A*x + B*u
%          y = C*x + D*u
%
% [*] G_tilt_ss:
% x = | tilt |; u = tiltc; y = tilt


Kp  = (pi/180)*(1/TILT_SCALE);  % rad/count

A = -1/Tp;
B = Kp/Tp;
C = 1;
D = 0;

try
    s         = tf('s');
    G_tilt    = Kp/(1+Tp*s);
    G_tilt_ss = ss(A, B, C, D);
catch
    G_tilt    = 0.0;
    G_tilt_ss = 0.0;
end

% % Example test code:
% % ------------------
% figure
% step(G_tilt*(180/pi))
% hold all
% step(G_tilt_ss*(180/pi))
% hold off

end


