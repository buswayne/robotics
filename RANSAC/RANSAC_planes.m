clear all
close all
%% Load Cloud Point
xyzPoints = load('pointCloud.mat');
ptCloud = xyzPoints.point;
figure
pcshow(ptCloud)
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
title('Original Point Cloud')
%% extract coordinates
x_coordinate = xyzPoints.point(:,:,1);
y_coordinate = xyzPoints.point(:,:,2);
z_coordinate = xyzPoints.point(:,:,3);
image_dimension = size(x_coordinate);
%% segmentation of Plane 1
fprintf('\n\nPlane 1:\n')
% initialize Plane 1 best variables
bestModel_1 = [];
bestInliers_1 = NaN(480,640,3);
bestOutliers_1 = NaN(480,640,3);
bestError_1 = inf;
ni_best_1 = 0;
no_best_1 = inf;
% custom parameters of the algorithm
threshold = 0.0125;
min_inliers = 50000;
iterations = 100;    

for it = 1:iterations
    fprintf('Current iteration: %i\n', it)
    inliers = NaN(480,640,3);                  %at every iteration reinitialize these values
    outliers = NaN(480,640,3);
    ni = 0;
    no = 0;
    totalError = 0;
    % select 3 random points within the ptCloud
    for r=1:3       
    point(r,:) = ptCloud(randi(image_dimension(1)),randi(image_dimension(2)),:);
    % if algorithm has randomly picked a NaN value, select again 
    while((isnan(point(r,1))) || isnan(point(r,2)) || isnan(point(r,3)))  %if the selected point is a NaN, select another point
        point(r,:) = ptCloud(randi(image_dimension(1)),randi(image_dimension(2)),:);
    end
    end
    x1 = point(1,1);    y1 = point(1,2);    z1 = point(1,3);
    x2 = point(2,1);    y2 = point(2,2);    z2 = point(2,3);
    x3 = point(3,1);    y3 = point(3,2);    z3 = point(3,3); 
    % to find a, b, c, d coeff. of the plane, use the cross product of two
    % vectors and passage through one point.
    v1 = point(1,:)-point(2,:);
    v2 = point(1,:)-point(3,:);
    cp = cross(v1,v2);                      %find vector plane
    a_est = cp(1);
    b_est = cp(2);
    c_est = cp(3);
    d = point(1,:)*[a_est b_est c_est]';    %find d coeff.

    % divide dataset into inliers and outliers
    for i=1:image_dimension(1)
        for j=1:image_dimension(2)
            while((isnan(x_coordinate(i,j)) || isnan(y_coordinate(i,j)) || isnan(z_coordinate(i,j))) && j<image_dimension(2))
                j = j + 1;            %if (i,j) has one NaN coordinate, skip it
            end
            dist = abs(a_est*x_coordinate(i,j)+b_est*y_coordinate(i,j)+c_est*z_coordinate(i,j)-d)/sqrt((a_est)^2+(b_est)^2+(c_est)^2);
            error = dist;
            if(dist < threshold)      %(i,j) is an inlier!
                totalError = totalError + error;
                inliers(i,j,:) = ptCloud(i,j,:); %add element to vector of inliers elements
                ni = ni+1;
            else                      %(i,j) is an outlier!
                outliers(i,j,:) = xyzPoints.point(i,j,:); %add element to vector of outliers elements
                no = no+1;
            end
        end
    end
    if (ni > min_inliers)                       % if the current model has enough inliers
        if ( totalError < bestError_1 )         % if the current model has the highest number of inliers
            a_best_1 = a_est;
            b_best_1 = b_est;
            c_best_1 = c_est;
            bestModel_1 = [a_best_1/d; b_best_1/d; c_best_1/d];
            bestInliers_1 = inliers;
            bestOutliers_1 = outliers;
            bestError_1 = totalError;
            ni_best_1 = ni;
            no_best_1 = no;
            fprintf('Found potential plane with %i inliers!\n', ni_best_1)
        end 
    end
    if (ni_best_1 > 80000)
        break
    end
end
fprintf('\n\nEquation of Plain 1: (%i)x+(%i)y+(%i)z = 1\n', bestModel_1)
%% display Plane 1 and ptCloud after Plane 1 removal
plane_1 = bestInliers_1;
figure
subplot(1,2,1)
pcshow(plane_1)
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
title('Plane 1')
remain_ptCloud = ptCloud;
for i=1:480
    for j=1:640
        while((isnan(bestInliers_1(i,j,1)) || isnan(bestInliers_1(i,j,2)) || isnan(bestInliers_1(i,j,3))) && j < 640 )
            j = j+1;
        end
        remain_ptCloud(i,j,:) = NaN;
    end
end
subplot(1,2,2)
pcshow(remain_ptCloud)
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
title('Point Cloud after Plane 1 removal')

%% segmentation of Plane 2
fprintf('\n\nPlane 2:\n')
% reinitialize ptCloud as the remain_ptCloud
ptCloud = remain_ptCloud;
x_coordinate = ptCloud(:,:,1);
y_coordinate = ptCloud(:,:,2);
z_coordinate = ptCloud(:,:,3);
% initialize Plane 2 best variables
bestModel_2 = [];
bestInliers_2 = NaN(480,640,3);
bestOutliers_2 = NaN(480,640,3);
bestError_2 = inf;
ni_best_2 = 0;
no_best_2 = inf;
% custom parameters of the algorithm
threshold = 0.0125;
min_inliers = 50000;
iterations = 100;    

for it = 1:iterations
    fprintf('Current iteration: %i\n', it)
    inliers = NaN(480,640,3);                  %at every iteration reinitialize these values
    outliers = NaN(480,640,3);
    ni = 0;
    no = 0;
    totalError = 0;
    % select 3 random points within the ptCloud
    for r=1:3       
    point(r,:) = ptCloud(randi(image_dimension(1)),randi(image_dimension(2)),:);
    % if algorithm has randomly picked a NaN value, select again 
    while((isnan(point(r,1))) || isnan(point(r,2)) || isnan(point(r,3)))  %if the selected point is a NaN, select another point
        point(r,:) = ptCloud(randi(image_dimension(1)),randi(image_dimension(2)),:);
    end
    end
    x1 = point(1,1);    y1 = point(1,2);    z1 = point(1,3);
    x2 = point(2,1);    y2 = point(2,2);    z2 = point(2,3);
    x3 = point(3,1);    y3 = point(3,2);    z3 = point(3,3); 
    % to find a, b, c, d coeff. of the plane, use the cross product of two
    % vectors and passage through one point.
    v1 = point(1,:)-point(2,:);
    v2 = point(1,:)-point(3,:);
    cp = cross(v1,v2);                      %find vector plane
    a_est = cp(1);
    b_est = cp(2);
    c_est = cp(3);
    d = point(1,:)*[a_est b_est c_est]';    %find d coeff.

    % divide dataset into inliers and outliers
    for i=1:image_dimension(1)
        for j=1:image_dimension(2)
            while((isnan(x_coordinate(i,j)) || isnan(y_coordinate(i,j)) || isnan(z_coordinate(i,j))) && j<image_dimension(2))
                j = j + 1;            %if (i,j) has one NaN coordinate, skip it
            end
            dist = abs(a_est*x_coordinate(i,j)+b_est*y_coordinate(i,j)+c_est*z_coordinate(i,j)-d)/sqrt((a_est)^2+(b_est)^2+(c_est)^2);
            error = dist;
            if(dist < threshold)      %(i,j) is an inlier!
                totalError = totalError + error;
                inliers(i,j,:) = ptCloud(i,j,:); %add element to vector of inliers elements
                ni = ni+1;
            else                      %(i,j) is an outlier!
                outliers(i,j,:) = xyzPoints.point(i,j,:); %add element to vector of outliers elements
                no = no+1;
            end
        end
    end
    if (ni > min_inliers)                       % if the current model has enough inliers
        if ( totalError < bestError_2 )         % if the current model has the highest number of inliers
            a_best_2 = a_est;
            b_best_2 = b_est;
            c_best_2 = c_est;
            bestModel_2 = [a_best_2/d; b_best_2/d; c_best_2/d];
            bestInliers_2 = inliers;
            bestOutliers_2 = outliers;
            bestError_2 = totalError;
            ni_best_2 = ni;
            no_best_2 = no;
            fprintf('Found potential plane with %i inliers!\n', ni_best_2)
        end 
    end
    if (ni_best_2 > 80000)
        break
    end
end
fprintf('\n\nEquation of Plain 2: (%i)x+(%i)y+(%i)z = 1\n', bestModel_2)
%% display Plane 2 and ptCloud after Plane 1, Plane 2 removal
plane_2 = bestInliers_2;
figure
subplot(1,2,1)
pcshow(plane_2)
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
title('Plane 2')
for i=1:480
    for j=1:640
        while((isnan(bestInliers_2(i,j,1)) || isnan(bestInliers_2(i,j,2)) || isnan(bestInliers_2(i,j,3))) && j < 640 )
            j = j+1;
        end
        remain_ptCloud(i,j,:) = NaN;
    end
end
subplot(1,2,2)
pcshow(remain_ptCloud)
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
title('Point Cloud after Plane 1, Plane 2 removal')

%% segmentation of Plane 3
fprintf('\n\nPlane 3:\n')
% reinitialize ptCloud with the remain_ptCloud
ptCloud = remain_ptCloud;
x_coordinate = ptCloud(:,:,1);
y_coordinate = ptCloud(:,:,2);
z_coordinate = ptCloud(:,:,3);
% initialize Plane 3 best variables
bestModel_3 = [];
bestInliers_3 = NaN(480,640,3);
bestOutliers_3 = NaN(480,640,3);
bestError_3 = inf;
ni_best_3 = 0;
no_best_3 = inf;
% custom parameters of the algorithm
threshold = 0.016;
min_inliers = 50000;
iterations = 100;    

for it = 1:iterations
    fprintf('Current iteration: %i\n', it)
    inliers = NaN(480,640,3);                  %at every iteration reinitialize these values
    outliers = NaN(480,640,3);
    ni = 0;
    no = 0;
    totalError = 0;
    % select 3 random points within the ptCloud
    for r=1:3       
    point(r,:) = ptCloud(randi(image_dimension(1)),randi(image_dimension(2)),:);
    % if algorithm has randomly picked a NaN value, select again 
    while((isnan(point(r,1))) || isnan(point(r,2)) || isnan(point(r,3)))  %if the selected point is a NaN, select another point
        point(r,:) = ptCloud(randi(image_dimension(1)),randi(image_dimension(2)),:);
    end
    end
    x1 = point(1,1);    y1 = point(1,2);    z1 = point(1,3);
    x2 = point(2,1);    y2 = point(2,2);    z2 = point(2,3);
    x3 = point(3,1);    y3 = point(3,2);    z3 = point(3,3); 
    % to find a, b, c, d coeff. of the plane, use the cross product of two
    % vectors and passage through one point.
    v1 = point(1,:)-point(2,:);
    v2 = point(1,:)-point(3,:);
    cp = cross(v1,v2);                      %find vector plane
    a_est = cp(1);
    b_est = cp(2);
    c_est = cp(3);
    d = point(1,:)*[a_est b_est c_est]';    %find d coeff.

    % divide dataset into inliers and outliers
    for i=1:image_dimension(1)
        for j=1:image_dimension(2)
            while((isnan(x_coordinate(i,j)) || isnan(y_coordinate(i,j)) || isnan(z_coordinate(i,j))) && j<image_dimension(2))
                j = j + 1;            %if (i,j) has one NaN coordinate, skip it
            end
            dist = abs(a_est*x_coordinate(i,j)+b_est*y_coordinate(i,j)+c_est*z_coordinate(i,j)-d)/sqrt((a_est)^2+(b_est)^2+(c_est)^2);
            error = dist;
            if(dist < threshold)      %(i,j) is an inlier!
                totalError = totalError + error;
                inliers(i,j,:) = ptCloud(i,j,:); %add element to vector of inliers elements
                ni = ni+1;
            else                      %(i,j) is an outlier!
                outliers(i,j,:) = xyzPoints.point(i,j,:); %add element to vector of outliers elements
                no = no+1;
            end
        end
    end
    if (ni > min_inliers)                       % if the current model has enough inliers
        if ( totalError < bestError_3 )         % if the current model has the highest number of inliers
            a_best_3 = a_est;
            b_best_3 = b_est;
            c_best_3 = c_est;
            bestModel_3 = [a_best_3/d; b_best_3/d; c_best_3/d];
            bestInliers_3 = inliers;
            bestOutliers_3 = outliers;
            bestError_3 = totalError;
            ni_best_3 = ni;
            no_best_3 = no;
            fprintf('Found potential plane with %i inliers!\n', ni_best_3)
        end 
    end
    if (ni_best_3 > 80000)
        break
    end
end
fprintf('\n\nEquation of Plain 3: (%i)x+(%i)y+(%i)z = 1\n', bestModel_3)
%% display Plane 3 and final ptCloud after Plane 1, Plane 2, Plane 3 removal
plane_3 = bestInliers_3;
figure
subplot(1,2,1)
pcshow(plane_3)
title('Plane 3')
for i=1:480
    for j=1:640
        while((isnan(bestInliers_3(i,j,1)) || isnan(bestInliers_3(i,j,2)) || isnan(bestInliers_3(i,j,3))) && j < 640 )
            j = j+1;
        end
        remain_ptCloud(i,j,:) = NaN;
    end
end
subplot(1,2,2)
pcshow(remain_ptCloud)
title('Point Cloud after Plane 1, Plane 2, Plane 3 removal')


%% cylinder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% THIS PART IS ONLY ADDITIONAL TO IMPLEMENT EXERCISE 4.2 %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% WITHOUT THE NEED TO DEFINE A ROI %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ptCloud = pointCloud(remain_ptCloud);
maxDistance = 0.005;
cylinder_axis = [0,0,1];
[model, inliers, outliers] = pcfitcylinder(ptCloud,maxDistance,cylinder_axis);
cylinder = select(ptCloud,inliers);
figure
pcshow(cylinder)
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
title('Cylinder')
fprintf('\n\nPosition of the cylinder:\n %i\n%i\n%i\n', model.Center);
fprintf('\n\nPose of the cylinder:\n %i\n%i\n%i\n', model.Orientation);
fprintf('\n\nRadius of the cylinder: %i\n', model.Radius);
fprintf('\n\nHeight of the cylinder: %i\n', model.Height);