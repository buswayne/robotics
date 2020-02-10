clear all
close all
%% QUIZ: COMPUTATION OF PHASE, POINT 2
BUSETTO_phase_shift_1;
% corresponding pixels in image plane 
f_x = 1/1920;           %compute frequency of the sinusoidal pattern: from given projected pattern
f_y = 1/1080;           %you can notice that T = image dimension (both for x and y) --> f = 1/T = 1/dimension
x_p = phase_x./(f_x*2*pi);
y_p = phase_y./(f_y*2*pi);
                               