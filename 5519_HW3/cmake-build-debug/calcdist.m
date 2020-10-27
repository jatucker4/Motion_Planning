function dist = calcdist(vec1,vec2)
%CALCDIST Summary of this function goes here
%   Detailed explanation goes here
x1 = vec1(1,1);
y1 = vec1(1,2);
x2 = vec2(1,1);
y2 = vec2(1,2);
dist = sqrt((x2-x1)^2 + (y2-y1)^2);
end

