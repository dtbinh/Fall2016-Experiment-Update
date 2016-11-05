% Main.m
clear
load('exp02vid36.mat');
counter = 1;

dt = 0.1

%% Global Parameter
% Scr_para;

%% Initialize System
Scr_init;

%% Loop
while(1)
	counter = counter + 1;
	Scr_frame;
	Scr_trig;
end

%% Clean Up
Scr_clean;