%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;

pt = [0;4]; % target point (in column matrix)

figure(1)
title('Planar Robot','FontSize',12)
axis([-20 20 -20 20])       % axis limits
axis manual                 % set axis to exact manual value(
hold on                     % does not erase previous graphs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% initialization of data
num_of_link = 3; %number of link
xdata = (0:num_of_link);
for i = 1:num_of_link+1
    ydata(i) = 0;
end
for i = 1:num_of_link
    angledata(i) = 0;
end

hnd = plot(xdata,ydata,'-r', 'erasemode','xor');    % plot robot manipulator

error = dist([xdata(num_of_link+1) ydata(num_of_link+1)], pt);
while (error > 0.5)
    iteration = num_of_link + 1;
    while (iteration > 1)
        %% CCD Algorthm

        pe = [xdata(num_of_link+1); ydata(num_of_link+1)];
        pc = [xdata(iteration-1); ydata(iteration-1)];

        a = (pe - pc)/norm(pe-pc);
        b = (pt - pc)/norm(pt-pc);
        teta = acosd(dot(a, b));
       
        direction = cross([a(1) a(2) 0],[b(1) b(2) 0]);
        if direction(3) < 0
            teta = -teta;
        end
       
        % this part can be omitted, this purpose is to make the result
        % looks more natural
        if (teta > 30)
            teta = 30;
        elseif (teta < -30)
            teta = -30;
        end
       
        angledata(iteration-1) = teta;

        iteration = iteration - 1;

        %% let's do the rotation!
        for i = 1:num_of_link-1
            R = [cosd(angledata(i)) -sind(angledata(i)); sind(angledata(i)) cosd(angledata(i))]; % rotation matrix
            % p' = R * (p - c) + c
            temp = R * ([xdata(i+1); ydata(i+1)] - [xdata(i); ydata(i)]) + [xdata(i); ydata(i)];
            xdata(i+1) = temp(1);
            ydata(i+1) = temp(2);
            angledata(i+1) = angledata(i+1) + angledata(i);
            xdata(i+2) = xdata(i+1)+1;
            ydata(i+2) = ydata(i+1);
        end

        % end effector rotation
        i = i+1;
        R = [cosd(angledata(i)) -sind(angledata(i)); sind(angledata(i)) cosd(angledata(i))]; % rotation matrix
        % p' = R * (p - c) + c
        temp = R * ([xdata(i+1); ydata(i+1)] - [xdata(i); ydata(i)]) + [xdata(i); ydata(i)];
        xdata(i+1) = temp(1);
        ydata(i+1) = temp(2);

    end
    error = dist([xdata(num_of_link+1) ydata(num_of_link+1)], pt);
    disp(error);
end

for i = 1:1:size(xdata,2)
    plot(xdata(i), ydata(i), 'ob')
end