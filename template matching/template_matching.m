clc
clear all
close all
%% import images
L = im2double(imread('left.png'));
R = im2double(imread('right.png'));
%% define target pixels
Imagesize = size(L);
y_target_pixel = [150,60,200];                                              %vector containing desired target pixels
x_target_pixel = [150,60,260];
target_pixel_number = numel(y_target_pixel);
%% define fixed window containing target pixel
% fixed window dimensions (SELECT AN EVEN NUMBER)
% (PLEASE INPUT AS MANY DIMENSIONS AS THE NUMBER OF TARGET PIXELS. Ex: 3 target pixels --> 3 y_side values, 3 x_side values )
y_side = [100,60,80];
x_side = [90,80,30];
% define left, upper corners of the fixed windows: with this formulation, you
% are sure that the fixed window contains the target pixel
y_starting_point = y_target_pixel-(y_side/2); 
x_starting_point = x_target_pixel-(x_side/2);
%% define relative position of target pixels in the fixed window
y_target_pixel_rel = y_target_pixel - y_starting_point + 1;
x_target_pixel_rel = x_target_pixel - x_starting_point + 1;
%% Plot left image with target pixels and fixed windows
figure
imagesc(L); hold on; plot(x_target_pixel,y_target_pixel,'r*'); title('Left image: desired target pixels and fixed windows visualization')
hold on;
for i=1:target_pixel_number
    rectangle('Position',[x_starting_point(i) y_starting_point(i) x_side(i) y_side(i)],'EdgeColor','r');
end

for i=1:target_pixel_number
    Fixedwindow = L(y_starting_point(i):(y_starting_point(i) + y_side(i) - 1),x_starting_point(i):(x_starting_point(i) + x_side(i) -1),:);
    
    % display the fixed windows and the target pixels
    figure
    imagesc(Fixedwindow); hold on; plot(x_target_pixel_rel(i),y_target_pixel_rel(i),'r*');
    %% check pixel correspondence between absolute position of target pixels and relative position in the correspondent fixed window
    % result should be a 3x1 vector of zeros
    L(y_target_pixel(i),x_target_pixel(i),:)-Fixedwindow(y_target_pixel_rel(i),x_target_pixel_rel(i),:)
end
%% Define sliding window in the right image
% do as many iterations as the number of declared target pixels 
for k=1:target_pixel_number
Fixedwindow = L(y_starting_point(k):(y_starting_point(k) + y_side(k) - 1),x_starting_point(k):(x_starting_point(k) + x_side(k) -1),:);
sliding_window_size = size(Fixedwindow);
Slidingwindow = R(1:sliding_window_size(1),1:sliding_window_size(2),:);   %initialize sliding window at the top-left of the right image
lowest_similarityscore = inf;                                             %initialize lowest_similarity_score in order to be sure it gets update at the first iteration of the loop
% define loop to make the sliding window move in the right image
for m = 1:(Imagesize(1)-sliding_window_size(1))                           %move along y direction
     for n = 1:(Imagesize(2)-sliding_window_size(2))                      %move along x direction
        Slidingwindow = R(m:(m+sliding_window_size(1)-1),n:(n+sliding_window_size(2)-1),:);
        % compare pixels of sliding window and fixed window
        for i = 1:sliding_window_size(1)                                  %loop to scroll along y direction
            for j = 1:sliding_window_size(2)                              %loop to scroll along x direction
                for l = 1:3                                               %loop to compare correspondent RGB value
                 % use one of the three method to assign a value at each couple of pixels 
                 value(i,j,l) = abs(Slidingwindow(i,j,l)-Fixedwindow(i,j,l));
                 % value (i,j,l) = sqrt((Slidingwindow(i,j,l)-Fixedwindow(i,j,l))^2);
                 % value (i,j,l) = abs(Slidingwindow(i,j,l)-Fixedwindow(i,j,l));
                end
            end
        end
        % compute similarity score of the overall sliding window in
        % position (m,n)
        similarityscore = sum(value, 'all');
        % similarityscore = max(value);
        if similarityscore <= lowest_similarityscore
            lowest_similarityscore = similarityscore;
            similarity_matrix = value;                                    %store the whole matrix with lowest ss
            %store position of the best window
            s_pos = m;
            r_pos = n;
        end
    end
end
%% Display results
Bestwindow = R(s_pos:(s_pos+sliding_window_size(1)-1),r_pos:(r_pos+sliding_window_size(2)-1),:);
% plot a figure for each target pixel. On the right image, the best window
% and the found pixel are plotted in green. Just for comparison, also
% target pixel and fixed window are plotted in red, with the same
% coordinate that they have in the left image: you can see how best window
% is shifted towards left along x direction
figure
subplot(1,2,1), imshow(R); 
hold on; plot(x_target_pixel(k),y_target_pixel(k),'r*');                                                            %position of target pixels (in the left image) seen in the right image
hold on; plot(r_pos+x_target_pixel_rel(k)-1,s_pos+y_target_pixel_rel(k)-1,'g*');                                    %position of target pixels in the right image
hold on; rectangle('Position',[x_starting_point(k) y_starting_point(k) x_side(k) y_side(k)],'EdgeColor','r');       %position of fixed window seen in the right image
hold on; rectangle('Position',[r_pos s_pos x_side(k) y_side(k)],'EdgeColor','g');                                   %position of best window in the right image
subplot(1,2,2), imagesc(Bestwindow); hold on; plot(x_target_pixel_rel(k),y_target_pixel_rel(k),'g*');
%% check pixel correspondence between absolute position of target pixels and relative position in the correspondent fixed window
% result should be a 3x1 vector of zeros
L(y_target_pixel(k),x_target_pixel(k),:) - R(s_pos+y_target_pixel_rel(k)-1,r_pos+x_target_pixel_rel(k)-1,:)
Fixedwindow(y_target_pixel_rel(k),x_target_pixel_rel(k),:) - Bestwindow(y_target_pixel_rel(k),x_target_pixel_rel(k),:)

end