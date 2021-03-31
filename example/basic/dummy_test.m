clear;
clc;
close all;
format




wgs84 = wgs84Ellipsoid();

xEast = 355.6013*1000;
yNorth = -923.0832*1000;
zUp = 1.0410e+03*1000;

lat0 = 45.9132;
lon0 = 36.7484;
h0 = 1877.7532*1000;


[x,y,z] = enu2ecef(xEast,yNorth,zUp,lat0,lon0,h0,wgs84)

XYZ = ch_ENU2ECEF(xEast, yNorth, zUp, deg2rad(lat0), deg2rad(lon0), h0);

XYZ


