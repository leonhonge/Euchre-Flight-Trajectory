%driver script for running modified Newtonian function with given .stl file

clear
clc
close all

%Geometry to import, must be .stl file
geometry = 'Bunny_Bomb - Revolve2.stl';
%Axis to slice down to get 2D slice, use checkPlanarSlice to determine
slice_axis = 'x';
%Plane offset if middle of vehicle is not already at 0, use
%checkPlanarSlice to determine
plane_offset = 0;
%Rotate if needed, use checkPlanarSlice to determine
rotate = 90;

%segs is N x 4 array with first two rows being coordinates of first vertice
%in segment and second two rows being coordinates of next vertice
%normals is N x 2 array with normal vectors for each segment
[segs, normals] = sliceSTLPlane(geometry, slice_axis, plane_offset, rotate);

%reference length used to determine CL and CD, can be used for
%recalculating lift and drag after
%NOTE: to get reference area multiply by average width of vehicle for now,
%will later include in function way to get total reference area for ease of
%use
refLength = max(segs(:,1)) - min(segs(:,1));

%angle of attack in degrees
alpha = 10;

%freestream Mach number
Mach = 5;

%Cp_vec is N x 1 vector of Cp values that can be used for debugging if shit
%seems off
[CL, CD, Cp_vec] = modifiedNewtonian(segs, normals, alpha, Mach, refLength);

disp("CL at " + alpha + " angle of attack and M = " + Mach)
disp(CL)
disp("CD at " + alpha + " angle of attack and M = " + Mach)
disp(CD)

%function to plot the vehicle Cp at each segment by multiplying the normal
%vector with the Cp at that location for visualization

plotVehicleCp(segs, normals, Cp_vec)