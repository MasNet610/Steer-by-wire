%% Steer By Wire system modeling
%% Steering wheel System
% System variables

syms th_sw dth_sw ddth_sw th_m1 dth_m1 ddth_m1 i_m1 di_m1 V_m1 x s

syms J_sw K_sw B_sw J_m1 B_m1 C_spr Nm d_l K_spr d_p N_starts N_spr T_load k_tm1 k_em1 R_m1 L_m1
syms M_motor_gear J_motor_gear J_linear G_PLA l_shaft r_bushing M_shaft grav_cons u_PLA
% Symbolic equation derivation
% *Steering wheel EOM*

ddth_sw = (1/J_sw)*(T_load - B_sw*dth_sw - C_spr*th_sw - K_sw*th_sw)
% Motor current equation

di_m1 = (1/L_m1)*(-R_m1*i_m1 - k_em1*dth_m1 + V_m1)

T_motor = k_tm1*i_m1;
%% 
% *Steering motor EOM*

ddth_m1 = -(C_spr + K_sw)*th_m1/(Nm^2 * J_m1) - (B_sw/Nm^2 + B_m1)/J_m1*dth_m1 + T_motor/J_m1
% Symbolic TFs
% These were used for PID control

TF_m1_current = -(s*k_em1*th_m1 + V_m1)/(s*L_m1 + R_m1)
TF_motor1 = (k_tm1/(s*L_m1 + R_m1))/ (J_m1*s^2 + (C_spr + K_sw)/(Nm^2) + (B_sw/(Nm^2) + B_m1 + (k_tm1*k_em1)/(s*L_m1 + R_m1))*s)
% State Space equations

x = [th_m1 dth_m1 i_m1]
u = [V_m1]

A_m1 = [0 1 0; -(C_spr + K_sw)/(J_m1*Nm^2) (-(B_m1 + B_sw/Nm^2)/J_m1) k_tm1/J_m1; 0 k_em1/L_m1 -R_m1/L_m1]
B_m1 = [0 ; 0 ; -1/L_m1]
C_m1 = eye(3);
D_m1 = zeros(3,1);
% Variables
% Variable list

G_PLA = 3.5e9; %PLA shear modulus
l_shaft = 0.24671219; %shaft length
M_motor_gear = 0.08116938; %Motor gear mass
J_motor_gear = 0.00004883; %Motor gear moment of inertia
J_linear = 0.00128838; %Moment of inertia of colinear shaft components
N_m1 = 2.3; %Gearing ratio. 1 turn of SW is 2.3 of Motor
k_tm1 = 0.641333333; % Motor constant
R_m1 = 14.8; %Motor resistance
L_m1 = 0.0444; % Motor inductance
k_em1 = 0.38; %Back emf constant
J_m1 = 10e-6; %Motor inertia
B_m1 = 0.00159; %Motor damping
u_PLA = 0.35; %High estimated friction coefficient 
B_sw = 0; %Shaft damping
M_shaft = 0.57593742; %rotating shaft mass
Nm = 2.3; %Gear ratio
K_spr = 80; % Spring constant (0.08 N/mm)
N_spr = 8; %Number of springs
N_starts = 2; %Screw starts
d_p = 0.008; % Screw pitch
r_bushing = 0.017; %Bushing radius
r_sliding = ((0.017/2)+5.5e-3); %friction radius for shaft slider
grav_cons = 9.8; %gravitational acceleration
s = tf('s');
% Variable calculation
% Calculation of spring constant C_spr

d_l = d_p * N_starts; % Screw lead calculation
x = d_l * (th_sw/(2*pi())); %spring compression distance
F_spr = (N_spr*(K_spr*(x))); %Force due to spring compression
T_spr = (F_spr * d_l)/(2*pi());

C_spr = (K_spr*N_spr*N_starts^2*d_p^2)/(4*(pi^2))
%% 
% Shaft moment of inertia

distance = (0.07396740/2) + (0.16500420/2);%radii of the two meshed gears
J_motor_gear_offset = J_motor_gear + M_motor_gear*distance^2; %parallel axis theorem
J_sw = J_linear + (J_motor_gear_offset) %Shaft moment of inertia 
%% 
% Shaft stiffness is based on rotation friction

F_spring_friction = u_PLA*F_spr; %The friction force due to the spring acting
F_normal_shaft = M_shaft * grav_cons; %Normal force of the shaft
F_shaft_friction = u_PLA*F_normal_shaft; %Friction due to shaft rotation
K_sw = (F_shaft_friction*r_bushing)+(F_spring_friction*r_sliding);
K_sw = double(subs(K_sw, th_sw, 1))
% Valued equation derivation
% Valued TFs
% These were used for PID control

TF_motor1 = (k_tm1/(s*L_m1 + R_m1))/ (J_m1*s^2 + (C_spr + K_sw)/(Nm^2) + (B_sw/(Nm^2) + B_m1 + (k_tm1*k_em1)/(s*L_m1 + R_m1))*s)
% State Space values

x = [th_m1 dth_m1 i_m1]

A_m1 = [0 1 0; -(C_spr + K_sw)/(J_m1*Nm^2) (-(B_m1 + B_sw/Nm^2)/J_m1) k_tm1/J_m1; 0 k_em1/L_m1 -R_m1/L_m1]
B_m1 = [0 ; 0 ; -1/L_m1]
C_m1 = eye(3);
D_m1 = zeros(3,1);

Q_m1 = eye(3);
R_m1 = 1;

mul = 1.5e2; %1e5 safe
mul_R = 1;
        %X Y TH DX DY DTH
Q_m1 = Q_m1.*[1*mul, 0.01*mul, 0.01*mul];
R_m1 = R_m1.*mul_R;

Klqr_m1 = lqr(A_m1,B_m1,Q_m1,R_m1)

A_cl_m1 = A_m1-B_m1*Klqr_m1;

K_dc_m1 = D_m1 - C_m1*inv(A_cl_m1)*B_m1
%% 
% 
%% SA system
% System variables

syms V_m2 dth_m2 th_m2 ddth_m2 y dy ddy s th_w dth_w ddth_w i_m2 di_m2 ddi_m2

syms B_r M_r k_lf r_L r_p B_kp I_f R_m2 L_m2 K_tm2 J_m2 B_m1 E K_r ia_m2 dia_m2 J_w k_em2
% Symbolic equations
% Motor EOM
% Paper "Development of estimation force feedback torque control algorithm for 
% driver steering feel in vehicle steer by wire System: Hardware in the loop"

ddth_m2 = -B_m1*dth_m2/J_m2 + K_r*ia_m2/J_m2

dia_m2 = -R_m2*ia_m2/L_m2 + K_tm2*dth_m2/L_m2 + V_m2/L_m2
% Rack and tire transfer function
% Paper "Development of estimation force feedback torque control algorithm for 
% driver steering feel in vehicle steer by wire System: Hardware in the loop"

ddy =(1/M_r)*((-2*k_lf*y/r_L^2)-(K_r*y/r_p^2)-B_r*dy + (K_r*th_m2/r_p))

ddth_w = (1/I_f)*((-k_lf*th_w + (k_lf*y/r_L) - B_kp*dth_w))
% Symbolic TFs

TF_motor2 = simplify(K_r/ (J_m2*(s*L_m2+R_m2)*s^2 + (B_m1*(s*L_m2+R_m2) - K_tm2*K_r)*s))

TF_rack = simplify((K_r/(r_p*M_r))/(s^2 + B_r*s/M_r + (2*k_lf/(M_r*r_L^2) + K_r/(r_p^2*M_r))))

TF_Wheel = simplify((k_lf/r_L) / (J_w*s^2 + B_kp*s + k_lf))
% State Space equations
% Motor SS

x = [th_m2 dth_m2 i_m2]
u = [V_m2]

A_m1 = [0 -(B_m1/J_m2) K_r/J_m2; 0 1 0; 0 -k_em2/L_m1 -R_m2/L_m2]
B_m1 = [0 ; 0 ; 1/L_m1]
C_m1 = eye(3);
D_m1 = zeros(3,1);
%% 
% Rack SS

x = [y dy]
u = [th_m2]

A_m1 = [0 1; (1/M_r)*((-2*k_lf/r_L^2)-(K_r/r_p^2)) -(B_r/M_r)]
B_m1 = [0 ; K_r/(M_r*r_p)]
C_m1 = eye(2);
D_m1 = zeros(2,1);
%% 
% Tire SS

x = [th_w dth_w]
u = [y]

A_m1 = [0 1; -(k_lf/J_w) -(B_kp/J_w)]
B_m1 = [0 ; k_lf/(J_w*r_L)]
C_m1 = eye(2);
D_m1 = zeros(2,1);
% Variables
% Variable list

s = tf('s');
R_m2 = 8.5; %Motor resistance
L_m2 = 0.0425; %Motor inductance
K_tm2 = 0.39667; %Motor constant
k_em2 = 1.64; %Back emf constant
J_m2 = 0.00001; %Motor inertia
B_m1 = 0.00583; %Motor damping

E = 2340000000; %Youngs modulus

r_p = 0.05216750/2; %Pinion gear radius
%K_r = 30.96; %Lumped torque stiffness (skeptical)
M_r = 0.101; %Rack mass
B_r = 0.003; %Estimated rack damping coefficient due to friction
k_lf = 0.00062; %Rack linkage stiffness (Ball bearings)
B_kp = 0.001; %Kingpin damping coefficient (PLA friction)
r_L = 0.04007805; %Offset of kingpin axis
% Variable calculation
% Lumped torque stiffness

F_rack_lumped_normal = 0.20928123 * grav_cons;%Normal force of the rack system
F_rack_friction = u_PLA*F_rack_lumped_normal; %Friction due to shaft rotation
K_r = (F_shaft_friction*r_p)
%% 
% Wheel inertia

J_w = 0.00058442; %Lumped front weel inertia
M_wheel = 0.29270232; %Wheel mass
J_w = J_w + M_wheel*(-0.00644323 - 0.02750000)^2 %parallel axis theorem

% Valued TFs

TF_motor2_valued = simplify(K_r/ (J_m2*(s*L_m2+R_m2)*s^2 + (B_m1*(s*L_m2+R_m2) - K_tm2*K_r)*s))
 
TF_rack_valued = simplify((K_r/(r_p*M_r))/(s^2 + B_r*s/M_r + (2*k_lf/(M_r*r_L^2) + K_r/(r_p^2*M_r))))

TF_Wheel_valued = simplify((k_lf/r_L) / (J_w*s^2 + B_kp*s + k_lf))
% State Space equations
% Motor SS

x = [th_m2 dth_m2 i_m2]
u = [V_m2]

A_m1 = [0 -(B_m1/J_m2) K_r/J_m2; 0 1 0; 0 -k_em2/L_m1 -R_m2/L_m2]
B_m1 = [0 ; 0 ; 1/L_m1]
C_m1 = eye(3);
D_m1 = zeros(3,1);
%% 
% Rack SS

x = [y dy]
u = [th_m2]

A_m1 = [0 1; (1/M_r)*((-2*k_lf/r_L^2)-(K_r/r_p^2)) -(B_r/M_r)]
B_m1 = [0 ; K_r/(M_r*r_p)]
C_m1 = eye(2);
D_m1 = zeros(2,1);
%% 
% Tire SS

x = [th_w dth_w]
u = [y]

A_m1 = [0 1; -(k_lf/J_w) -(B_kp/J_w)]
B_m1 = [0 ; k_lf/(J_w*r_L)]
C_m1 = eye(2);
D_m1 = zeros(2,1);