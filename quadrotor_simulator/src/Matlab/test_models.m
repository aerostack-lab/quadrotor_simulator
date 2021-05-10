clc
clear all
close all

%% Test Yaw model:
%  ---------------
DYAW_SCALE = 2047 / 254.760; % counts/deg/s, rc/controller input
Tp = 0.5;     % seconds , desired peak time
Zeta = 0.80;  % 0<Zeta<1, adimensional, desired damping coeffcient

[A B C D G_Y_ss G_Y A_dY B_dY C_dY D_dY G_dY G_dY_ss] = create_yaw_model( DYAW_SCALE, Tp, Zeta);


[dY0 T0_dY0g] = step((180/pi)*G_dY);
[Y0  T0_Y0g ] = step((180/pi)*G_Y, 1);
[Yallss T0ss] = step((180/pi)*G_Y_ss, 1);
Y0ss  = Yallss(:,1);
dY0ss = Yallss(:,2);
% plot dY
figure()
plot(T0_dY0g,dY0)
hold all
plot(T0ss,dY0ss)
hold off
% plot Y
figure()
plot(T0_Y0g,Y0)
hold all
plot(T0ss,Y0ss)
hold 

clear DYAW_SCALE Tp Zeta;
clear A B C D G_Y_ss G_Y A_dY B_dY C_dY D_dY G_dY G_dY_ss;
clear dY0ss T0_Y0g T0_dY0g T0ss Y0 Y0ss Yallss dY0;

%% Test Tilt model:
%  ----------------
TILT_SCALE = 40;   % counts/deg, rc/controller input
Tp         = 0.10; % seconds , desired response time

[A, B, C, D, G_tilt_ss, G_tilt] = create_tilt_model( TILT_SCALE, Tp);


figure
step(G_tilt*(180/pi))
hold all
step(G_tilt_ss*(180/pi))
hold off

clear TILT_SCALE Tp;
clear A B C D G_tilt_ss G_tilt;

%% Test z model:
%  -------------

THRUST_SCALE = 4095 / 32; % counts/N - approximate, rc/controller input
Tth   = 0.15/3;           % sec, response time of propellers
m     = 1.5;              % kg, estimated mass of the UAV
vzmax = 2.5;              % m/s
pitch_sym = 0; roll_sym = 0; % not a symbolic model

[A, B, C, D, G_z_ss] = create_z_model( THRUST_SCALE, Tth, m, vzmax, pitch_sym, roll_sym);


thrust = 15*     (4095/32); % "x" [N] * gain
[Zallss T0ss] = step(G_z_ss, 1);
Z_T  = Zallss(:,1,1)*thrust;
dZ_T = Zallss(:,2,1)*thrust;
Z_p  = Zallss(:,1,2);
dZ_p = Zallss(:,2,2);
% plot Z
figure
plot(T0ss,Z_T+Z_p)
% plot dZ
figure
plot(T0ss,dZ_T+dZ_p)

clear THRUST_SCALE Tth m pitch_sym roll_sym vzmax;
clear G_z_ss A B C D;
clear thrust, T0ss, Z_T, Z_p, Zallss, dZ_T, dZ_p

%% Test with symbolic variables
clear all

syms DYAW_SCALE Tp_Y Zeta_Y;
[A_Y, B_Y, C_Y, D_Y] = create_yaw_model( DYAW_SCALE, Tp_Y, Zeta_Y);
syms PITCH_SCALE Tp_P ROLL_SCALE Tp_R;
[A_P, B_P, C_P, D_P] = create_tilt_model( PITCH_SCALE, Tp_P);
[A_R, B_R, C_R, D_R] = create_tilt_model(  ROLL_SCALE, Tp_R);
syms THRUST_SCALE Tth m vzmax pitch_sym roll_sym;
[A_z, B_z, C_z, D_z] = create_z_model( THRUST_SCALE, Tth, m, vzmax, pitch_sym, roll_sym);