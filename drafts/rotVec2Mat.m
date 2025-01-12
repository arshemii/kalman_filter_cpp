function [rotMat] = rotVect2Mat(W)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
rotMat = [0   -W(3)   0
           W(3)    0    -W(3)
           -W(2)    W(1)     0];
end

