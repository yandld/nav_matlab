function [ center, radii, evecs, v, chi2 ] = ellipsoid_fit_new( X, equals )
%
% Fit an ellispoid/sphere/paraboloid/hyperboloid to a set of xyz data points:
%
%   [center, radii, evecs, pars ] = ellipsoid_fit( X )
%   [center, radii, evecs, pars ] = ellipsoid_fit( [x y z] );
%   [center, radii, evecs, pars ] = ellipsoid_fit( X, 1 );
%   [center, radii, evecs, pars ] = ellipsoid_fit( X, 2, 'xz' );
%   [center, radii, evecs, pars ] = ellipsoid_fit( X, 3 );
%
% Parameters:
% * X, [x y z]   - Cartesian data, n x 3 matrix or three n x 1 vectors
% * flag         - '' or empty fits an arbitrary ellipsoid (default),
%                - 'xy' fits a spheroid with x- and y- radii equal
%                - 'xz' fits a spheroid with x- and z- radii equal
%                - 'xyz' fits a sphere
%                - '0' fits an ellipsoid with its axes aligned along [x y z] axes
%                - '0xy' the same with x- and y- radii equal
%                - '0xz' the same with x- and z- radii equal
%
% Output:
% * center    -  ellispoid or other conic center coordinates [xc; yc; zc]
% * radii     -  ellipsoid or other conic radii [a; b; c]
% * evecs     -  the radii directions as columns of the 3x3 matrix
% * v         -  the 10 parameters describing the ellipsoid / conic algebraically: 
%                Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz + J = 0
% * chi2      -  residual sum of squared errors (chi^2), this chi2 is in the 
%                coordinate frame in which the ellipsoid is a unit sphere.
%
% Author:
% Yury Petrov, Oculus VR
% Date:
% September, 2015
%

%reference 
narginchk( 1, 3 ) ;  % check input arguments
if nargin == 1
    equals = ''; % no constraints by default
end
    
if size( X, 2 ) ~= 3
    error( 'Input data must have three columns!' );
else
    x = X( :, 1 );
    y = X( :, 2 );
    z = X( :, 3 );
end

% need nine or more data points
if length( x ) < 9 && strcmp( equals, '' ) 
   error( 'Must have at least 9 points to fit a unique ellipsoid' );
end
if length( x ) < 8 && ( strcmp( equals, 'xy' ) || strcmp( equals, 'xz' ) )
   error( 'Must have at least 8 points to fit a unique ellipsoid with two equal radii' );
end
if length( x ) < 6 && strcmp( equals, '0' )
   error( 'Must have at least 6 points to fit a unique oriented ellipsoid' );
end
if length( x ) < 5 && ( strcmp( equals, '0xy' ) || strcmp( equals, '0xz' ) )
   error( 'Must have at least 5 points to fit a unique oriented ellipsoid with two equal radii' );
end
if length( x ) < 4 && strcmp( equals, 'xyz' );
   error( 'Must have at least 4 points to fit a unique sphere' );
end

% fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx +
% 2Hy + 2Iz + J = 0 and A + B + C = 3 constraint removing one extra
% parameter
if strcmp( equals, '' )
    D = [ x .* x + y .* y - 2 * z .* z, ...
        x .* x + z .* z - 2 * y .* y, ...
        2 * x .* y, ...
        2 * x .* z, ...
        2 * y .* z, ...
        2 * x, ...
        2 * y, ...
        2 * z, ...
        1 + 0 * x ];  % ndatapoints x 9 ellipsoid parameters
elseif strcmp( equals, 'xy' )
    D = [ x .* x + y .* y - 2 * z .* z, ...
        2 * x .* y, ...
        2 * x .* z, ...
        2 * y .* z, ...
        2 * x, ...
        2 * y, ...
        2 * z, ...
        1 + 0 * x ];  % ndatapoints x 8 ellipsoid parameters
elseif strcmp( equals, 'xz' )
    D = [ x .* x + z .* z - 2 * y .* y, ...
        2 * x .* y, ...
        2 * x .* z, ...
        2 * y .* z, ...
        2 * x, ...
        2 * y, ...
        2 * z, ...
        1 + 0 * x ];  % ndatapoints x 8 ellipsoid parameters
    % fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Gx + 2Hy + 2Iz = 1
elseif strcmp( equals, '0' )
    D = [ x .* x + y .* y - 2 * z .* z, ...
          x .* x + z .* z - 2 * y .* y, ...
          2 * x, ...
          2 * y, ... 
          2 * z, ... 
          1 + 0 * x ];  % ndatapoints x 6 ellipsoid parameters
    % fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Gx + 2Hy + 2Iz = 1,
    % where A = B or B = C or A = C
elseif strcmp( equals, '0xy' )
    D = [ x .* x + y .* y - 2 * z .* z, ...
          2 * x, ...
          2 * y, ... 
          2 * z, ... 
          1 + 0 * x ];  % ndatapoints x 5 ellipsoid parameters
elseif strcmp( equals, '0xz' )
    D = [ x .* x + z .* z - 2 * y .* y, ...
          2 * x, ...
          2 * y, ... 
          2 * z, ... 
          1 + 0 * x ];  % ndatapoints x 5 ellipsoid parameters
     % fit sphere in the form A(x^2 + y^2 + z^2) + 2Gx + 2Hy + 2Iz = 1
elseif strcmp( equals, 'xyz' )
    D = [ 2 * x, ...
          2 * y, ... 
          2 * z, ... 
          1 + 0 * x ];  % ndatapoints x 4 ellipsoid parameters
else
    error( [ 'Unknown parameter value ' equals '!' ] );
end

% solve the normal system of equations
d2 = x .* x + y .* y + z .* z; % the RHS of the llsq problem (y's)
u = ( D' * D ) \ ( D' * d2 );  % solution to the normal equations

% find the residual sum of errors
% chi2 = sum( ( 1 - ( D * u ) ./ d2 ).^2 ); % this chi2 is in the coordinate frame in which the ellipsoid is a unit sphere.

% find the ellipsoid parameters
% convert back to the conventional algebraic form
if strcmp( equals, '' )
    v(1) = u(1) +     u(2) - 1;
    v(2) = u(1) - 2 * u(2) - 1;
    v(3) = u(2) - 2 * u(1) - 1;
    v( 4 : 10 ) = u( 3 : 9 );
elseif strcmp( equals, 'xy' )
    v(1) = u(1) - 1;
    v(2) = u(1) - 1;
    v(3) = -2 * u(1) - 1;
    v( 4 : 10 ) = u( 2 : 8 );
elseif strcmp( equals, 'xz' )
    v(1) = u(1) - 1;
    v(2) = -2 * u(1) - 1;
    v(3) = u(1) - 1;
    v( 4 : 10 ) = u( 2 : 8 );
elseif strcmp( equals, '0' )
    v(1) = u(1) +     u(2) - 1;
    v(2) = u(1) - 2 * u(2) - 1;
    v(3) = u(2) - 2 * u(1) - 1;
    v = [ v(1) v(2) v(3) 0 0 0 u( 3 : 6 )' ];

elseif strcmp( equals, '0xy' )
    v(1) = u(1) - 1;
    v(2) = u(1) - 1;
    v(3) = -2 * u(1) - 1;
    v = [ v(1) v(2) v(3) 0 0 0 u( 2 : 5 )' ];
elseif strcmp( equals, '0xz' )
    v(1) = u(1) - 1;
    v(2) = -2 * u(1) - 1;
    v(3) = u(1) - 1;
    v = [ v(1) v(2) v(3) 0 0 0 u( 2 : 5 )' ];
elseif strcmp( equals, 'xyz' )
    v = [ -1 -1 -1 0 0 0 u( 1 : 4 )' ];
end
v = v';

% form the algebraic form of the ellipsoid
A = [ v(1) v(4) v(5) v(7); ...
      v(4) v(2) v(6) v(8); ...
      v(5) v(6) v(3) v(9); ...
      v(7) v(8) v(9) v(10) ];
% find the center of the ellipsoid
center = -A( 1:3, 1:3 ) \ v( 7:9 );
% form the corresponding translation matrix
T = eye( 4 );
T( 4, 1:3 ) = center';
% translate to the center
R = T * A * T';
% solve the eigenproblem
[ evecs, evals ] = eig( R( 1:3, 1:3 ) / -R( 4, 4 ) );
radii = sqrt( 1 ./ diag( abs( evals ) ) );
sgns = sign( diag( evals ) );
radii = radii .* sgns;

% calculate difference of the fitted points from the actual data normalized by the conic radii
d = [ x - center(1), y - center(2), z - center(3) ]; % shift data to origin
d = d * evecs; % rotate to cardinal axes of the conic;
d = [ d(:,1) / radii(1), d(:,2) / radii(2), d(:,3) / radii(3) ]; % normalize to the conic radii
chi2 = sum( abs( 1+0*x - sum( d.^2, 2 ) ) );
 
if abs( v(end) ) > 1e-6
    v = -v / v(end); % normalize to the more conventional form with constant term = -1
else
    v = -sign( v(end) ) * v;
end




