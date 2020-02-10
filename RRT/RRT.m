
%% Rapidly-Exploring Random Trees (RRT)

%% Initialization
close all
clear all
%% Define map
obs = false(25,25);
obs(7:18,16:18) = true;
obs(7,7:18) = true;
figure,imshow(obs);

%% Convert obs to map object
gridMap = robotics.BinaryOccupancyGrid(obs,1);
figure, show(gridMap);

%% 
statesMin = [gridMap.XWorldLimits(1); gridMap.YWorldLimits(1)];
statesMax = [gridMap.XWorldLimits(2); gridMap.YWorldLimits(2)];
stepSize = 1; % \epsilon

%% Define start and goal
startState = [5;5];
goalState = [20;20];
hold on;
plot(startState(1), startState(2),'o');
plot(goalState(1), goalState(2),'o');
goalCirc = 0.5;
viscircles(goalState',goalCirc);

%% ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% RRT algorithm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization
% parameters
EPS = stepSize;
K = 5000;
threshold = 0.5;

% initialize tree
q_start.coord = startState';
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = goalState';
q_goal.cost = 0;

% Initialize the tree with q_start
%((1) G.add.vertex(q_init))
nodes = q_start;

%% Start the loop
for it=1:K
    % ((2.1) Random Selected Vertex)
    q_rand = [floor(rand(1)*25) floor(rand(1)*25)]; 
    %plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410])

    % ((2.2) Nearest_Vertex(G,q_rand))
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);


    % ((2.3) q_add computation)
    q_add.coord = steer(q_rand, q_near.coord, val, EPS);
    q_add.cost = dist(q_add.coord, q_near.coord) + q_near.cost;
    q_add.parent = idx;

    if (detect_collision(q_add.coord, q_near.coord))
        % Skip to next iteration
        continue
    else
    % Add the q_add to the tree
    %((3) G.add.vertex(q_add) + G.add.edge(q_add,q_n))
        nodes = [nodes q_add];
        line([q_near.coord(1), q_add.coord(1)], [q_near.coord(2), q_add.coord(2)], 'Color', 'b', 'LineWidth', 1);
        drawnow
        hold on
    end



    % Compute closest node in the tree to q_goal
    %((4) Nearest Vertex(G, q_goal))
    ndist = [];
    for j = 1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_goal.coord);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near_star = nodes(idx);            


    %((5) If Distance(q*, q_goal) < threshold)
    if (dist(q_near_star.coord, q_goal.coord) < threshold)
        fprintf("\n\nFound Path!\nIteration:%i\n\n", it)
        
        % Additional Absolute Least Cost Path (GREEN)
        q_curr = q_near_star;
        while (q_curr.coord ~= q_start.coord)
            q_curr.parent = assign_parent(nodes, q_curr, nodes(q_curr.parent));
            q_curr = nodes(q_curr.parent); 
        end

        % Shortest path (RED)
        q_goal.parent = idx;
        q_end = q_goal;
        nodes = [nodes q_goal];
        while q_end.parent ~= 0
            start = q_end.parent;
            line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)],'Color','r','LineWidth', 2);
            hold on
            q_end = nodes(start);
        end


        break 
    end
    it
end

if (it >= K)
    disp('No Path found. Increase number of iterations and retry.');
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% FUNCTIONS SECTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% distance
function d = dist(q1,q2)
d = sqrt((q1(1)-q2(1))^2 + (q1(2)-q2(2))^2);
end
        
        
        
  
%% steer
function A = steer(qr, qn, val, eps)
   qnew = [0 0];
   
   % Steer towards qn with maximum step size of eps
   if val >= eps
       qnew(1) = qn(1) + ((qr(1)-qn(1))*eps)/dist(qr,qn);
       qnew(2) = qn(2) + ((qr(2)-qn(2))*eps)/dist(qr,qn);
   else
       qnew(1) = qr(1);
       qnew(2) = qr(2);
   end   
   A = [qnew(1), qnew(2)];
end



%% collision function
function exit_flag = detect_collision(q1,q2)

line = [q1(1) q1(2); q2(1) q2(2)];

%Define the obstacle region in world coordinate

poly1 = polyshape([15 15 18 18], [18 7 7 18]);
poly2 = polyshape([6 6 18 18],[19 18 18 19]);
poly = union(poly1,poly2);

[in,out] = intersect(poly,line);

if sum(in) == 0
    exit_flag = 0;
else
    exit_flag = 1;
end
end

%% assign parent function
% Assign parent with minimum distance from q_add
function parent = assign_parent(graph, node, node_near)
        q_nearest = [];
        neighbor_count = 1;
        for j = 1:length(graph)
            if (detect_collision(graph(j).coord, node.coord)==0)
                q_nearest(neighbor_count).coord = graph(j).coord;
                q_nearest(neighbor_count).cost = graph(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end
        
        % Initialize cost to currently known value
        q_min = node_near;
        C_min = node.cost;
    
        % Iterate through all nearest neighbors to find alternate lower
        % cost paths
        
        for k = 1:length(q_nearest)
            if (q_nearest(k).cost + dist(q_nearest(k).coord, node.coord) < C_min)
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist(q_nearest(k).coord, node.coord);                
                hold on
            end
        end
        line([q_min.coord(1), node.coord(1)], [q_min.coord(2), node.coord(2)], 'Color', 'g','LineWidth',2);
        % Update parent to least cost-from node
        for j = 1:1:length(graph)
            if (graph(j).coord == q_min.coord)
                parent = j;
            end
        end
end