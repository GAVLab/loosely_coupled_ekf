clear,clc,close all

syms N E D vN vE vD phi theta psi bfx bfy bfz bgx bgy bgz ...
     gyrx gyry gyrz ... 
     ox oy oz

pos = [N;E;D];
vel = [vN;vE;vD];
att = [phi;theta;psi];

Cbn = RotationMatrix(att);

x = [N E D vN vE vD phi theta psi bfx bfy bfz bgx bgy bgz].';

% --- Position measurement
r_ba_b = [ox;oy;oz]; % Antenna offset in the vehicle body frame

r_b_n = x(1:3); % position of the vehicle in the nav frame

r_ba_n = Cbn*r_ba_b;

r_a_n = r_b_n + r_ba_n;

% --- Velocity measurement
Omega = [gyrx;gyry;gyrz] - x(13:15);

v_b_n = x(4:6); % velocity of the CG resolved in the nav frame 

v_a_n = v_b_n + cross(Omega,r_ba_n);

h_pos = r_a_n;
h_vel = v_a_n;
h = [h_pos;h_vel];
H = jacobian(h,x); % (??)

H = simplify(H);


%% Syms subs
n=length(x);

for i=1:3
    att_{i,1} = ['att(',num2str(i),')'];
end

H = subs(H,att,att_);

Hpos = H(1:3,:);
Hvel = H(4:6,:);






