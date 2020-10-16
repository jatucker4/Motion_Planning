clc; clear all; close all;
%% Plot Rectangles
OriginCoords = importdata("Kinematics_1_LinkLocs.txt");
first_vertex = OriginCoords(1:2,1);
second_vertex = OriginCoords(1:2,2);
third_vertex = OriginCoords(2:3,1);
fourth_vertex = OriginCoords(2:3,2);
endEffector1 = OriginCoords(3:4,1);
endEffector2 = OriginCoords(3:4,2);

figure
hold on
plot(first_vertex, second_vertex,'LineWidth',3);
plot(third_vertex,fourth_vertex,'LineWidth',3);
plot(endEffector1,endEffector2,'LineWidth',3);
plot(second_vertex(1),second_vertex(2),'b*','LineWidth',5);
plot(endEffector1(1),endEffector1(2),'k*','LineWidth',5);
title("Forward Kinematics With $\theta_1$=0,$\theta_2$=45,$\theta_3$=45 Degrees",...
    'Interpreter','latex','FontSize',16)
legend("Link1","Link2","Link3","Origin(0,0)","End Effector(17.0711,17.0711)")
xlabel("X-Axis")
ylabel("Y-Axis")
xlim([-1 20])
ylim([-1 20])
%% Q7 Part 2
OriginCoords2 = importdata("Kinematics_22_LinkLocs.txt");
first_vertex = OriginCoords2(1:2,1);
second_vertex = OriginCoords2(1:2,2);
third_vertex = OriginCoords2(2:3,1);
fourth_vertex = OriginCoords2(2:3,2);
endEffector1 = OriginCoords2(3:4,1);
endEffector2 = OriginCoords2(3:4,2);

figure
hold on
plot(first_vertex, second_vertex,'LineWidth',3);
plot(third_vertex,fourth_vertex,'LineWidth',3);
plot(endEffector1,endEffector2,'LineWidth',3);
plot(0,0,'b*','LineWidth',5);
plot(10,20,'k*','LineWidth',5);
title("Inverse Kinematics With End Effector at: $x$=10,$y$=20",...
    'Interpreter','latex','FontSize',16)
legend("Link1","Link2","Link3","Origin","End Effector")
xlabel("X-Axis")
ylabel("Y-Axis")
xlim([-1 15])
ylim([-1 22])
angles = importdata("Kinematics_2_LinkLocs.txt");
fprintf("The three angles were found to be:\n");
fprintf("theta1 = %d radians, theta2 = %d radians, theta3 = %d radians\n",angles(1),angles(2),angles(3));
%% Q8 Part a
resolution = 500^2;
TriangleCSpace = importdata("CSpace_Part_a.txt");
xCoords = TriangleCSpace(1:resolution,1);
yCoords = TriangleCSpace(1:resolution,2);
obsBools = TriangleCSpace((resolution+1):end,1);

for i = 1:resolution
    if(obsBools(i) == 1)
        colorMap(i,:) = [1,0,0];
    else
        colorMap(i,:) = [0,0,1];
    end
end
% Plot the CSpace
figure
scatter(xCoords,yCoords,24.*ones(resolution,1),colorMap,'filled','Marker','s')
xlim([0,2*pi]);
ylim([0.025133,6.258]);
title("C-Space for Question 8 Part a")
xlabel("Theta1[rad]")
ylabel("Theta2[rad]")

% Plot the Workspace
figure
xLocs = [-0.25,0,0.25];
yLocs = [0.25,0.75,0.25];
plot(0,0,'r*')
grid on
title("Workspace for Question 8 Part a")
xlim([-2,2])
ylim([-2,2])
patch(xLocs,yLocs,'k')
legend('Manipulator Base','Obstacle')
xlabel("X-Axis")
ylabel("Y-Axis")
%% Q8 Part b
TriangleCSpaceb = importdata("CSpace_Part_b.txt");
xCoordsb = TriangleCSpaceb(1:resolution,1);
yCoordsb = TriangleCSpaceb(1:resolution,2);
obsBoolsb = TriangleCSpaceb((resolution+1):end,1);
figure
hold on
for i = 1:resolution
    if(obsBoolsb(i) == 1)
        colorMapb(i,:) = [1,0,0];
    else
        colorMapb(i,:) = [0,0,1];
    end
end
scatter(xCoordsb,yCoordsb,24.*ones(resolution,1),colorMapb,'filled','Marker','s')
xlim([0,2*pi]);
ylim([0.025133,6.258]);
title("C-Space for Question 8 Part b")
xlabel("Theta1[rad]")
ylabel("Theta2[rad]")

%Plot the Workspace
figure
xLocs1 = [-0.25,-0.25,0.25,0.25];
yLocs1 = [1.1,2,2,1.1];
xLocs2 = [-2,-2,2,2];
yLocs2 = [-2,-1.8,-1.8,-2];
plot(0,0,'r*')
grid on
xlim([-2,2])
ylim([-2,2])
patch(xLocs1,yLocs1,'k')
patch(xLocs2,yLocs2,'k')
legend('Manipulator Base','Obstacle1','Obstacle2')
title("Workspace for Question 8 Part b")
xlabel("X-Axis")
ylabel("Y-Axis")
%% Q8 Part c
TriangleCSpacec = importdata("CSpace_Part_c.txt");
xCoordsc = TriangleCSpacec(1:resolution,1);
yCoordsc = TriangleCSpacec(1:resolution,2);
obsBoolsc = TriangleCSpacec((resolution+1):end,1);
figure
hold on
for i = 1:resolution
    if(obsBoolsc(i) == 1)
        colorMapc(i,:) = [1,0,0];
    else
        colorMapc(i,:) = [0,0,1];
    end
end
scatter(xCoordsc,yCoordsc,24.*ones(resolution,1),colorMapc,'filled','Marker','s')
xlim([0,2*pi]);
ylim([0.025133,6.258]);
title("C-Space for Question 8 Part c")
xlabel("Theta1[rad]")
ylabel("Theta2[rad]")

%Plot the Workspace
figure
xLocs = [-0.25,0,0.25];
yLocs = [0.25,0.75,0.25];
xLocs2 = [-2,-2,2,2];
yLocs2 = [-0.5,-0.3,-0.3,-0.5];
plot(0,0,'r*')
grid on
xlim([-2,2])
ylim([-2,2])
patch(xLocs,yLocs,'k')
patch(xLocs2,yLocs2,'k')
legend('Manipulator Base','Obstacle1','Obstacle2')
title("Workspace for Question 8 Part c")
xlabel("X-Axis")
ylabel("Y-Axis")

