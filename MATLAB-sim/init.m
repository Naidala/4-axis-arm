close all
clc

scale = 1;      % now in mm

d1 = 50*scale;
a1 = 0;
a2 = 240*scale;
a3 = 160*scale;

k = [0.5,0.5,0.5];
K = diag(k);

par = [a1, a2, a3, d1];
q0 = [deg2rad(0); deg2rad(90); deg2rad(-90)];

des_E = [20*scale; 140*scale; 290*scale]; % POSIZIONE DESIDERATA DELL'END-EFFECTOR
start_E = [0*scale; 160*scale; 290*scale];

% des_E = [0.16*scale; 0.20*scale; 0.29*scale];
% des_E = [60*scale; -80*scale; 100*scale];

Ts = 0.1;