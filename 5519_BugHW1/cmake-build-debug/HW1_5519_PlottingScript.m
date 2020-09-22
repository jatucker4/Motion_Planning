close all;
clear all;
clc;

%% Create the obstacles for the first scenario
Rec1 = [1,1,1,4];
Rec2 = [3,4,1,8];
Rec3 = [6,5,6,1];
Rec4 = [12,5,1,8];
Rec5 = [3,12,9,1];
PathCoords = importdata("Bug1PathFile.txt");
PathCoords2 = importdata("Bug2PathFile.txt");

figure(1)
hold on
rectangle('Position',Rec1, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec2, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec3, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec4, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec5, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);

pz1 = plot(PathCoords(:,1),PathCoords(:,2),'Color',[0.75, 0, 0.75,0.3],'LineWidth',3);
pz2 = plot(0,0,'b*');
pz3 = plot(10,10,'g*');

legend([pz1,pz2,pz3],'Bug1 Path','qStart','qGoal');
title("Bug1 Scenario 1")
hold off

figure(2)
hold on
rectangle('Position',Rec1, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec2, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec3, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec4, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec5, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);

pz1 = plot(PathCoords2(:,1),PathCoords2(:,2),'Color',[0.75, 0, 0.75,0.3],'LineWidth',3);
pz2 = plot(0,0,'b*');
pz3 = plot(10,10,'g*');

legend([pz1,pz2,pz3],'Bug2 Path','qStart','qGoal');
title("Bug2 Scenario 1")
hold off

%% Begin scenario 2 Plots
Rec1 = [-6,-6,31,1];
Rec2 = [-6,-5,1,10];
Rec3 = [4,-5,1,6];
Rec4 = [14,-5,1,6];
Rec5 = [24,-5,1,6];
Rec6 = [-6,5,36,1];
Rec7 = [9,0,1,5];
Rec8 = [19,0,1,5];
Rec9 = [29,0,1,5];
PathCoords3 = importdata("Bug1Path2File.txt");
PathCoords4 = importdata("Bug2Path2File.txt");

figure(3)
hold on
rectangle('Position',Rec1, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec2, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec3, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec4, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec5, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec6, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec7, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec8, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);
rectangle('Position',Rec9, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor',[0, 0.4470, 0.7410]);

pz1 = plot(PathCoords3(:,1),PathCoords3(:,2),'Color',[0.75, 0, 0.75,0.3],'LineWidth',3);
pz2 = plot(0,0,'b*');
pz3 = plot(35,0,'g*');

legend([pz1,pz2,pz3],'Bug1 Path','qStart','qGoal');
title("Bug1 Scenario 2")
hold off

figure(4)
hold on
rectangle('Position',Rec1, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor','k');
rectangle('Position',Rec2, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor','k');
rectangle('Position',Rec3, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor','k');
rectangle('Position',Rec4, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor','k');
rectangle('Position',Rec5, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor','k');
rectangle('Position',Rec6, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor','k');
rectangle('Position',Rec7, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor','k');
rectangle('Position',Rec8, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor','k');
rectangle('Position',Rec9, 'FaceColor',	[0, 0.4470, 0.7410],'EdgeColor','k');

pz1 = plot(PathCoords4(:,1),PathCoords4(:,2),'Color',[0.75, 0, 0.75,0.3],'LineWidth',3);
pz2 = plot(0,0,'b*');
pz3 = plot(35,0,'g*');

legend([pz1,pz2,pz3],'Bug2 Path','qStart','qGoal');
title("Bug2 Scenario 2")
hold off