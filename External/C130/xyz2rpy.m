function [roll,pitch,yaw] = xyz2rpy(x,y,z,RollFactor)
%XYZ2RPY returns some estimated roll, pitch, and yaw for an airplane
%traveling along the path given by x,y,z.  This function is designed to be
%used with the c130 function.  
% 
%% Syntax & Description
% 
% [roll,pitch,yaw] = xyz2rpy(x,y,z) returns roll, pitch, and yaw in
% degrees for a path given by global coordinates x,y,z.  Inputs x, y, 
% and z can be vectors of matching lengths, or if a value is constant
% (e.g., if elevation does not change), a scalar value may be used.
% 
% [roll,pitch,yaw] = xyz2rpy(x,y,z,RollFactor) exaggerates or reduces
% roll by RollFactor. This may be linked to speed of the aircraft. If the airplane
% is moving very fast, you may want to use a RollFactor much greater
% than 1.  To fly a plane without any roll, set RollFactor to 0.
% RollFactor can be scalar or an array matching the dimensions of x.
% Default RollFactor is unity.  
% 
%% Author Info
% Chad A. Greene of the University of Texas Institute for Geophysics (UTIG)
% wrote this on Saturday, September 27, 2014. Feel free to visit Chad on
% over at http://www.chadagreene.com. 
% 
% See also c130. 

%% Input checks: 

assert(nargin>2,'xyz2rpy requires inputs x, y, and z.') 
assert(nargin<5,'Too many inputs to xyz2rpy.') 
assert(isvector(x)==1,'Input x must be a scalar or vector.')
assert(isvector(y)==1,'Input y must be a scalar or vector.')
assert(isvector(z)==1,'Input z must be a scalar or vector.')
assert(nargout==3,'xyz2rpy wants to give you roll, pitch, and yaw, so you will need to allow for 3 output arguments.')

nx = max([numel(x) numel(y) numel(z)]); % length of longest input
assert(nx>1,'All of your inputs are scalars.  At least one input needs to be a vector.') 

% Convert scalar inputs to vectors if necessary: 
if isscalar(x) 
    x = x*ones(nx,1); 
end
if isscalar(y)
    y = y*ones(nx,1); 
end
if isscalar(z)
    z = z*ones(nx,1); 
end
assert(numel(x)==numel(y)&&numel(y)==numel(z),'x,y,z must be vectors of matching dimensions. You can also enter one or two inputs as scalars. But you cannot do whatever you have just tried to do.')

if nargin==4
    assert(isnumeric(RollFactor)==1,'RollFactor must be numeric value.')
    if isscalar(RollFactor) 
        RollFactor = RollFactor*ones(nx,1); 
    else
        assert(numel(RollFactor)==numel(x),'RollFactor must be a scalar value or an array matching the dimensions of x.')
    end
else
    RollFactor = ones(nx,1); 
end
%% Calculations: 

% Calculate pitch as change in z over change in along-track distance:  
[~,dxdy]= gradient([x(:) y(:)]);
alongTrackDist = cumsum(hypot(dxdy(:,1),dxdy(:,2))); 
pitch = atand(gradient(z,alongTrackDist)); 

% Calculate yaw as instantaneous slope dy./dx minus 90 degrees
yaw = atan2(dxdy(:,2),dxdy(:,1))*(180/pi)-90; 

% Let roll be related to the change in yaw over distance flown: 
roll = -atand(RollFactor.*gradient(unwrap((yaw+90)*(pi/180)),alongTrackDist)); 

