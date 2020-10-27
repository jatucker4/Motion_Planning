clear all;close all; clc;
tcirc = linspace(0, 2*pi);
rcirc = 0.25;
xcirc = rcirc.*cos(tcirc);
ycirc = rcirc.*sin(tcirc);

%% Q2 PART A
xLocs1 = [3.5,3.5,4.5,4.5];
yLocs1 = [0.5,1.5,1.5,0.5];
xLocs2 = [6.5,6.5,7.5,7.5];
yLocs2 = [-1.5,-0.5,-0.5,-1.5];
BrushVars = load("BrushFire1.txt");
DiscVars = load("DiscGrid1.txt");
gradpath = load("GradPath.txt");
potFieldXY = load("PotentialField.txt");
arwlen = hypot(potFieldXY(:,1), potFieldXY(:,2));
part_a_dist = 0;
for i = 1:length(gradpath)-1
    part_a_dist = part_a_dist + calcdist(gradpath(i,:),gradpath(i+1,:));
    distcheck(i) = calcdist(gradpath(i,:),gradpath(i+1,:));
end
fprintf("Distance traveled in Q2 Part A: %f\n",part_a_dist);
% Create plots for Q2 Part A
figure(2)
hold on
h1 = plot(gradpath(:,1),gradpath(:,2),'r','LineWidth',1.5);
quiver(DiscVars(:,1),DiscVars(:,2),potFieldXY(:,1)./arwlen,potFieldXY(:,2)./arwlen);
xlim([0,10.25])
ylim([-3,3])
patch(xLocs1,yLocs1,'k')
patch(xLocs2,yLocs2,'k')
p1 = patch(10+xcirc,ycirc,'k');
p1.FaceAlpha = 0.25;
pz2 = plot(0,0,'b.','MarkerSize',30);
pz3 = plot(10,0,'g.','MarkerSize',30);
legend([h1,pz2,pz3],"Path To Goal","Start","Goal");
title("Gradient Planner For Question 2 Part a",'Interpreter','latex','FontSize',16)
xlabel("X-Axis",'Interpreter','latex','FontSize',12)
ylabel("Y-Axis",'Interpreter','latex','FontSize',12)
%% Q2 PART B
Rec1 = [1,1,1,4];
Rec2 = [3,4,1,8];
Rec3 = [6,5,6,1];
Rec4 = [12,5,1,8];
Rec5 = [3,12,9,1];
DiscVars1 = load("DiscGrid11.txt");
gradpath1 = load("GradPath1.txt");
potFieldXY1 = load("PotentialField1.txt");
arwlen1 = hypot(potFieldXY1(:,1), potFieldXY1(:,2));
part_b_dist = 0;
for i = 1:length(gradpath1)-1
    part_b_dist = part_b_dist + calcdist(gradpath1(i,:),gradpath1(i+1,:));
end
fprintf("Distance traveled in Q2 Part B Scenario 1: %f\n",part_b_dist);
% Plot the vector field and path
figure(3)
hold on
quiver(DiscVars1(:,1),DiscVars1(:,2),potFieldXY1(:,1)./arwlen1,potFieldXY1(:,2)./arwlen1);
xlim([0,13])
ylim([0,15])
rectangle('Position',Rec1, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec2, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec3, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec4, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec5, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
h1 = plot(gradpath1(:,1),gradpath1(:,2),"r-","LineWidth",2);
pz2 = plot(0,0,'b.','MarkerSize',20);
pz3 = plot(10,10,'g.','MarkerSize',20);
p1 = patch(10+xcirc,10+ycirc,'k');
p1.FaceAlpha = 0.25;
% Plot Labels
legend([h1,pz2,pz3],"Path To Goal","Start","Goal");
title("Gradient Planner For Question 2 Part b",'Interpreter','latex','FontSize',16)
xlabel("X-Axis",'Interpreter','latex','FontSize',12)
ylabel("Y-Axis",'Interpreter','latex','FontSize',12)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Rec1 = [-6,-6,31,1];
Rec2 = [-6,-5,1,10];
Rec3 = [4,-5,1,6];
Rec4 = [14,-5,1,6];
Rec5 = [24,-5,1,6];
Rec6 = [-6,5,36,1];
Rec7 = [9,0,1,5];
Rec8 = [19,0,1,5];
Rec9 = [29,0,1,5];
DiscVars2 = load("DiscGrid2.txt");
gradpath2 = load("GradPath2.txt");
potFieldXY2 = load("PotentialField2.txt");
part_b2_dist = 0;
for i = 1:length(gradpath2)-1
    part_b2_dist = part_b2_dist + calcdist(gradpath2(i,:),gradpath2(i+1,:));
end
fprintf("Distance traveled in Q2 Part B Scenario 2: %f\n",part_b2_dist);
arwlen2 = hypot(potFieldXY2(:,1), potFieldXY2(:,2));  
% Plot vector field
figure(4)
hold on
h1 = plot(gradpath2(:,1),gradpath2(:,2),'r','LineWidth',1.5);
quiver(DiscVars2(:,1),DiscVars2(:,2),potFieldXY2(:,1)./arwlen2,potFieldXY2(:,2)./arwlen2);
rectangle('Position',Rec1, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec2, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec3, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec4, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec5, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec6, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec7, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec8, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec9, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
% Plot Labels
xlim([-10,35.3])
ylim([-8,8])
pz2 = plot(0,0,'b.','MarkerSize',20);
pz3 = plot(35,0,'g.','MarkerSize',20);
p1 = patch(35+xcirc,ycirc,'k');
p1.FaceAlpha = 0.25;
legend([h1,pz2,pz3],'Path to Goal','qStart','qGoal');
title("Gradient Planner For Question 2 Part b",'Interpreter','latex','FontSize',16)
xlabel("X-Axis",'Interpreter','latex','FontSize',12)
ylabel("Y-Axis",'Interpreter','latex','FontSize',12)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3a
waveDiscVars = load("wavDisc1.txt");
waveGridVars = load("waveGrid1.txt");
wavePath = load("wavePath1.txt");
wavePath = [[0,0];[0,.042253];wavePath(1:end-2,:)];
wavePath = [wavePath;[9.9296,10];[10,10]];
wave_a_dist = 0;
for i = 1:length(wavePath)-1
    wave_a_dist = wave_a_dist + calcdist(wavePath(i,:),wavePath(i+1,:));
end
fprintf("Distance traveled in Q3 Part A Scenario 1: %f\n",wave_a_dist);
newvec1 = [waveDiscVars,waveGridVars(:,3)];
Rec1 = [1,1,1,4];
Rec2 = [3,4,1,8];
Rec3 = [6,5,6,1];
Rec4 = [12,5,1,8];
Rec5 = [3,12,9,1];

figure(5)
hold on
hplot1 = gridplot(newvec1);
h1 = plot(wavePath(:,1),wavePath(:,2),"r-","LineWidth",2);
pz2 = plot(0,0,'b.','MarkerSize',10);
pz3 = plot(10,10,'g.','MarkerSize',10);
h = colorbar;
set(get(h,'label'),'string','Manhattan Distance From Goal');
xlim([-3,15])
ylim([-3,15])
legend([h1,pz2,pz3],"Path To Goal","Start","Goal");
title("Cspace View of Path to Goal for Q8 Scenario 1",'Interpreter','latex','FontSize',16)
xlabel("X-Axis",'Interpreter','latex','FontSize',12)
ylabel("Y-Axis",'Interpreter','latex','FontSize',12)


figure(6)
hold on
grid on
rectangle('Position',Rec1, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec2, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec3, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec4, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec5, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
h1 = plot(wavePath(:,1),wavePath(:,2),"r-","LineWidth",2);
pz2 = plot(0,0,'b.','MarkerSize',10);
pz3 = plot(10,10,'g.','MarkerSize',10);
legend([h1,pz2,pz3],"Path To Goal","Start","Goal");
title("WorkSpace View of Path to Goal for Q8 Scenario 1",'Interpreter','latex','FontSize',16)
xlabel("X-Axis",'Interpreter','latex','FontSize',12)
ylabel("Y-Axis",'Interpreter','latex','FontSize',12)
%% 3b
waveDiscVars2 = load("wavDisc2.txt");
waveGridVars2 = load("waveGrid2.txt");
wavePath2 = load("wavePath2.txt");
wavePath2 = [[0,0];wavePath2];
wavePath2 = [wavePath2;[35,0]];
wave_b_dist = 0;
for i = 1:length(wavePath2)-1
    wave_b_dist = wave_b_dist + calcdist(wavePath2(i,:),wavePath2(i+1,:));
end
fprintf("Distance traveled in Q3 Part A Scenario 2: %f\n",wave_b_dist);
Rec1 = [-6,-6,31,1];
Rec2 = [-6,-5,1,10];
Rec3 = [4,-5,1,6];
Rec4 = [14,-5,1,6];
Rec5 = [24,-5,1,6];
Rec6 = [-6,5,36,1];
Rec7 = [9,0,1,5];
Rec8 = [19,0,1,5];
Rec9 = [29,0,1,5];
newvec2 = [waveDiscVars2,waveGridVars2(:,3)];
%
figure(7)
hold on
hplot2 = gridplot(newvec2);
h1 = plot(wavePath2(:,1),wavePath2(:,2),"r-","LineWidth",2);
h = colorbar;
set(get(h,'label'),'string','Manhattan Distance From Goal');
xlim([-10,35])
ylim([-7,7])
pz2 = plot(0,0,'b.','MarkerSize',10);
pz3 = plot(35,0,'g.','MarkerSize',10);
legend([h1,pz2,pz3],'Path to Goal','qStart','qGoal');
title("Cspace View of Path to Goal for Q8 Scenario 2",'Interpreter','latex','FontSize',16)
xlabel("X-Axis",'Interpreter','latex','FontSize',12)
ylabel("Y-Axis",'Interpreter','latex','FontSize',12)

figure(8)
hold on
grid on
rectangle('Position',Rec1, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec2, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec3, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec4, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec5, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec6, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec7, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec8, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec9, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
h1 = plot(wavePath2(:,1),wavePath2(:,2),"r-","LineWidth",2);
% Plot Labels
xlim([-10,35])
ylim([-8,8])
pz2 = plot(0,0,'b.','MarkerSize',10);
pz3 = plot(35,0,'g.','MarkerSize',10);
legend([h1,pz2,pz3],'Path to Goal','qStart','qGoal');
title("WorkSpace View of Path to Goal for Q8 Scenario 2",'Interpreter','latex','FontSize',16)
xlabel("X-Axis",'Interpreter','latex','FontSize',12)
ylabel("Y-Axis",'Interpreter','latex','FontSize',12)


%% Q4
waveDiscVars3 = load("wavDisc3.txt");
waveGridVars3 = load("waveGrid3.txt");
wavePath3 = load("wavePath3.txt");

newvec3 = [waveDiscVars3,waveGridVars3(:,3)];
figure(9)
hold on
hplot3 = gridplot(newvec3);
hz = plot(0,0,'b.','MarkerSize',30);
hx = plot(pi,0,'g.','MarkerSize',30);
h1 = plot(wavePath3(:,1),wavePath3(:,2),"r-","LineWidth",2);
h = colorbar;
set(get(h,'label'),'string','Manhattan Distance From Goal');
xlim([0, 2*pi])
ylim([0, 2*pi])
legend([h1,hz,hx],'Path to Goal','qStart','qGoal');
title("Cspace View of Path to Goal for Q8 Scenario 2",'Interpreter','latex','FontSize',16)
xlabel("$\theta_1$ (rad)",'Interpreter','latex','FontSize',12)
ylabel("$\theta_2s$ (rad)",'Interpreter','latex','FontSize',12)


figure(10)
hold on
OriginCoords = importdata("workPath1.txt");
for i = 1:3:length(OriginCoords)
    first_vertex = OriginCoords(i:i+1,1);
    second_vertex = OriginCoords(i:i+1,2);
    third_vertex = OriginCoords(i+1:i+2,1);
    fourth_vertex = OriginCoords(i+1:i+2,2);
    plot(first_vertex, second_vertex,'Color', [0.5 0.5 0.5],'LineWidth',3);
    plot(third_vertex,fourth_vertex,'Color', [0.5 0.5 0.5],'LineWidth',3);
    scatter(OriginCoords(i+2,1), OriginCoords(i+2,2), 50, 'r','filled', 'Marker', 's');
end

xLocs = [-0.25,-0.25,0.25,0.25];
yLocs = [1.1,2,2,1.1];
xLocs2 = [-2,-2,2,2];
yLocs2 = [-0.5,-0.3,-0.3,-0.5];
hz = plot(2,0,'b.','MarkerSize',30);
hx = plot(-2,0,'g.','MarkerSize',30);
grid on
xlim([-2,2])
ylim([-2,2])
patch(xLocs,yLocs,'k')
patch(xLocs2,yLocs2,'k')
legend([hz,hx],'Start','Goal')
title("Q8: Workspace Path of Manipulator",'Interpreter','latex','FontSize',16)
xlabel("X-Axis",'Interpreter','latex','FontSize',12)
ylabel("Y-Axis",'Interpreter','latex','FontSize',12)