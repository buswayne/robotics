close all
clear all
%% QUIZ: COMPUTATION OF PHASE, POINT 1
% x dimension
image_0=im2double(imread('imagex0.png'));
image_1=im2double(imread('imagex1.png'));
image_2=im2double(imread('imagex2.png'));
image_3=im2double(imread('imagex3.png'));
image_x_dimension=size(image_0);
phase_x=zeros(size(image_0));       %initialize the phase_x matrix with zero value for every pixel in the captured camera image
for i=1:image_x_dimension(1)        %for loop to scroll all the pixels in camera image
    for j=1:image_x_dimension(2)
        phase_x(i,j)=atan2((image_0(i,j)-image_2(i,j)),(image_1(i,j)-(image_3(i,j)))); 
        if(phase_x(i,j)<0)
            phase_x(i,j) = phase_x(i,j)+2*pi;  %given pixel intensity, compute phase of each pixels
        end
    end
end
figure
subplot(1,2,1)                      
imshow(phase_x)
   
% y dimension
image_y0=im2double(imread('imagey0.png'));
image_y1=im2double(imread('imagey1.png'));
image_y2=im2double(imread('imagey2.png'));
image_y3=im2double(imread('imagey3.png'));
image_y_dimension=size(image_y0);
phase_y=zeros(size(image_y0));        %initialize the phase_y matrix with zero value for every pixel in the captured camera image
for i=1:image_y_dimension(1)
    for j=1:image_y_dimension(2)
        phase_y(i,j)=atan2((image_y0(i,j)-image_y2(i,j)),(image_y1(i,j)-(image_y3(i,j))));   %add pi/2 to move atan from (-pi/2;pi/2) to (0,pi))
    if(phase_y(i,j)<0)
            phase_y(i,j) = phase_y(i,j)+2*pi;  %given pixel intensity, compute phase of each pixels
        end
    end
end
subplot(1,2,2)
imshow(phase_y)

%% QUIZ: COMPUTATION OF PHASE, POINT 2
% corresponding pixels in image plane 
f_x = 1/1920;           %compute frequency of the sinusoidal pattern: from given projected pattern
f_y = 1/1080;           %you can notice that T = image dimension (both for x and y) --> f = 1/T = 1/dimension
x_p = phase_x./(f_x*2*pi);
y_p = phase_y./(f_y*2*pi);
                               

