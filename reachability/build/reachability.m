clear; close all; clc;

%# read file contents: date,time,value
fil = readmatrix("reachability.csv");
width = 1;
x = fil(:,1);
y = fil(:,2);
z = fil(:,3);

%stem3(x,y,z, '-b','LineWidth',4);
%xlabel('x'); ylabel('y'); zlabel('z');
%zoom on; grid on;
 
scatterbar3(x,y,z,0.05)