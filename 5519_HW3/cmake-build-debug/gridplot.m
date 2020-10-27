function h_surf = gridplot(h, gd)
%grplot Plots the gridded data as a surface
% 
% gridplot(h, gd) displays a regularly gridded data gd as a
% surface in the axes h. The h parameter is optional, and the surface is
% plotted in the current axes, if it is not assigned. The function returns
% the handle of the created patch object.
% 
% Inputs:
%   h: handle of the axes in which the surface is plotted.
%   gd: m-by-3 matrix as (x, y, z) triplets. The z value is
%   assigned as a color to the corresponding patch. A random
%   value is assigned to z when the grid data is a m-by-2 matrix.
% Developed by: Mohammad Ali Goudarzi (ma.goudarzi@gmail.com)
% Version: 1.0.1
% Last update: March 25, 2014
% 
% Update log:
%     1.0.1: sort the input grid data (2014-03-25)
% Check the input arguments
narginchk(1, 2);
if nargin == 1
    gd = h;
    h = gca;
end
if size(gd, 2) == 2
    gd = [gd, rand(size(gd, 1), 1)];
end
% Sort the input data
gd = sortrows(gd, 1);
% Assign the corresponding x, y, and z values
x = gd(:, 1);
y = gd(:, 2);
z = gd(:, 3);
% Find the increment in x and y directions
if x(1) == x(2)
    ny = diff(find(y == y(1), 2));
    nx = numel(x) / ny;
elseif y(1) == y(2)
    nx = diff(find(x == x(1), 2));
    ny = numel(y) / nx;
end
dx = (x(end) - x(1)) / (nx - 1);
dy = (y(end) - y(1)) / (ny - 1);
% Make coordinates of the verices of the patches
x_vert = [x - dx / 2, x + dx / 2, x + dx / 2, x - dx / 2];
y_vert = [y - dy / 2, y - dy / 2, y + dy / 2, y + dy / 2];
% Plot the patches
h_surf = patch(x_vert', y_vert', z', 'Parent', h);
return;