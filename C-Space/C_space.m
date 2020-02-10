clear all
close all
%% robot definition
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


%% C-SPACE extraction
theta_0_axis = (-pi:0.05:pi);
theta_1_axis = (-pi:0.05:pi);

% main cycle 
figure
for i=1:length(theta_1_axis)        %along the vertical
    for j=1:length(theta_0_axis)    %along the horizontal
        q = [theta_0_axis(j)
             theta_1_axis(i)];
        link1.Pose  = robot.getTransform(q,'link1')*trvec2tform([L1/2,0,0]);
        link2.Pose  = robot.getTransform(q,'link2')*trvec2tform([L2/2,0,0]);
        isCollidedLink1 = checkCollision(link1,obstacle);
        isCollidedLink2 = checkCollision(link2,obstacle);
        % collision!
        if (isCollidedLink1 || isCollidedLink2)
            plot(theta_0_axis(j),theta_1_axis(i),'-s','MarkerEdgeColor','yellow','MarkerFaceColor','yellow')
            h = gca;
            h.YDir = 'reverse';
            hold on
            obs(j,i) = true;
        % no collision!
        else
            plot(theta_0_axis(j),theta_1_axis(i),'-s','MarkerEdgeColor','blue','MarkerFaceColor','blue')
            hold on
            h = gca;
            h.YDir = 'reverse';
        end
    end
end

