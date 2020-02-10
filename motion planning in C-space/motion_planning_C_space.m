%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% THERE ARE SOME BUGS IN THE ROBOTICS PACKAGE R2019 %%%%%%%%%%%
%%%%%%%%%%%%% RELATED TO FIGURES IF YOU RUN THE CODE AT ONCE! %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% PLEASE RUN THE CODE SECTION BY SECTION %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% TO HAVE THE CODE WORK FINE %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% robot definition
clear all
close all

robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

%parameters
L1 = 0.5;
L2 = 0.5;
pos_obs = [0.5,0.5,0.2]; %[x,y,r]

%definition base
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

%definition link 1
body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');

%definition link 2
body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L2, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');

%inverse kinematics
ik = inverseKinematics('RigidBodyTree',robot);
ik.SolverParameters.MaxIterations = 100;
ik.SolverParameters.GradientTolerance = 1e-3;
ik.SolverParameters.SolutionTolerance = 1e-2;
ik.SolverParameters.MaxTime = 0.01;
ik.SolverParameters.AllowRandomRestart = true;

%collision
link1 = collisionBox(L1,0.05,0.05);
link2 = collisionBox(L2,0.05,0.05);
obstacle = collisionSphere(pos_obs(3));

%initial configuration
q = robot.homeConfiguration;

%pose
link1.Pose  = robot.getTransform(q,'link1')*trvec2tform([L1/2,0,0]);
link2.Pose  = robot.getTransform(q,'link2')*trvec2tform([L2/2,0,0]);
obstacle.Pose = trvec2tform([pos_obs(1:2),0]);

%% obstacle definition

step = 0.05;
theta_0 = (-pi:step:pi);
theta_1 = (-pi:step:pi);

resolution = 1/step;

map = binaryOccupancyMap(2*pi, 2*pi, resolution);
map.GridOriginInLocal = [-pi -pi];

% main cycle 
figure
for i=1:length(theta_1)        %along the vertical
    for j=1:length(theta_0)    %along the horizontal      
        q = [theta_0(j)
             theta_1(i)];
        link1.Pose  = robot.getTransform(q,'link1')*trvec2tform([L1/2,0,0]);
        link2.Pose  = robot.getTransform(q,'link2')*trvec2tform([L2/2,0,0]);
        isCollidedLink1 = checkCollision(link1,obstacle);
        isCollidedLink2 = checkCollision(link2,obstacle);
        % collision!
        if (isCollidedLink1 || isCollidedLink2)
            setOccupancy(map, ([j i]/resolution-pi), 1);
        end
    end
end


inflate(map, 0.001)

figure
show(map)
h = gca;
h.YDir = 'reverse';
xlabel('Theta 0')
ylabel('Theta 1')


%% RRTStar

start = [-0.2, 1.3, 0];
goal = [1.8, -1.24, 0];

ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
sv.ValidationDistance = 0.01;
sv.Map = map;
ss.StateBounds =  [[-pi pi]; [-pi pi]; [-pi pi]];
planner = plannerRRTStar(ss,sv);
planner.MaxIterations = 10000;
planner.MaxConnectionDistance = 0.3;
planner.MaxNumTreeNodes = 1e10;
planner.ContinueAfterGoalReached = true;

%rng(100, 'twister') % repeatable result
[pthObj, solnInfo] = plan(planner,start,goal);
nodes = [pthObj.States; goal];

figure


show(map)
xlabel('Theta 0')
ylabel('Theta 1')
hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
plot(nodes(:,1),nodes(:,2),'r-','LineWidth',2); % draw path
hold off;
h = gca;
h.YDir = 'reverse';



%% show the corresponding path in x-y space

pose_vector = nodes(:,1:2);
x = zeros(length(pose_vector));
y = zeros(length(pose_vector));
figure
for i=1:length(pose_vector)
    q = pose_vector(i,:)';
    q1 = q(1);
    q2 = q(2);
    x(i) = L1*cos(q1)+L2*cos(q1+q2);
    y(i) = L1*sin(q1)+L2*sin(q1+q2);

    
    show(robot,q,'Frame','off');
    rotate3d off;
    view(2) 
    ax = gca;
    ax.Toolbar.Visible = 'off';
    ax.Projection = 'orthographic'; 
    hold on;
    [~,patchObj3] = show(obstacle);
    patchObj3.EdgeColor = 'none';
    patchObj3.FaceColor = 'blue';
    
    plot(x(i), y(i), 'or')
    
    

    if(i>1)
       line([x(i) x(i-1)], [y(i) y(i-1)], 'Color', 'r', 'LineWidth', 1);
    end
    pause(0.25);
  
end


%% direct kinematics
figure
show(robot,q,'PreservePlot',false,'Frame','off');
rotate3d off;
view(2) 
ax = gca;
ax.Toolbar.Visible = 'off';
ax.Projection = 'orthographic'; 
hold on;
[~,patchObj1] = show(link1);
[~,patchObj2] = show(link2);
[~,patchObj3] = show(obstacle);
patchObj1.EdgeColor = 'none';
patchObj2.EdgeColor = 'none';
patchObj3.EdgeColor = 'none';
patchObj1.FaceColor = 'green';
patchObj2.FaceColor = 'green';
patchObj3.FaceColor = 'blue';


pose_vector = nodes(:,1:2);
x = zeros(length(pose_vector));
y = zeros(length(pose_vector));

for i=1:length(pose_vector)
    q = pose_vector(i,:)';
    q1 = q(1);
    q2 = q(2);
    x(i) = L1*cos(q1)+L2*cos(q1+q2);
    y(i) = L1*sin(q1)+L2*sin(q1+q2);
    plot(x(i), y(i), 'or')
    link1.Pose  = robot.getTransform(q,'link1')*trvec2tform([L1/2,0,0]);
    link2.Pose  = robot.getTransform(q,'link2')*trvec2tform([L2/2,0,0]);
    show(robot,q,'PreservePlot',true,'Frame','off');
    delete(patchObj1);
    delete(patchObj2);
    delete(patchObj3);
    
    [~,patchObj1] = show(link1);
    [~,patchObj2] = show(link2);
    [~,patchObj3] = show(obstacle);

    patchObj1.EdgeColor = 'none';
    patchObj2.EdgeColor = 'none';
    patchObj3.EdgeColor = 'none';

    patchObj1.FaceColor = 'green';
    patchObj2.FaceColor = 'green';
    patchObj3.FaceColor = 'blue';
    

    if(i>1)
       line([x(i) x(i-1)], [y(i) y(i-1)], 'Color', 'r', 'LineWidth', 1);
    end
    pause(0.25);
  
end
