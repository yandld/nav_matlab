%% show mag cal result

function cal_res_3dplot_show(X, v)
%
% show raw data and cal ellposd
%
%   [center, radii, evecs, pars ] = ellipsoid_fit( X, v )
%
% Parameters:
% * X, [x y z]   - Cartesian data, n x 3 matrix or three n x 1 vectors
% * v, [v1; v2; v3...]  matrix, row is the 10 parameters describing the ellipsoid / conic
% algebraically:  Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz
% + J = 0.  usually 10x2 matrix, 10, parameters,  2 modules to compare
%                

    x = X( :, 1 );
    y = X( :, 2 );
    z = X( :, 3 );
    
    % color table
    coltbl = ['g' 'b' 'y', 'c', 'k'];
    
    figure,
    plot3( x, y, z, '.r' );
    hold on;

    mind = min( [ x y z ] );
    maxd = max( [ x y z ] );
    nsteps = 50;
    step = ( maxd - mind ) / nsteps;
    [ x, y, z ] = meshgrid( linspace( mind(1) - step(1), maxd(1) + step(1), nsteps ), linspace( mind(2) - step(2), maxd(2) + step(2), nsteps ), linspace( mind(3) - step(3), maxd(3) + step(3), nsteps ) );

    for  i=1:size(v, 1)
        m = v(i,:);
        Ellipsoid = m(1) *x.*x +   m(2) * y.*y + m(3) * z.*z + ...
                  2*m(4) *x.*y + 2*m(5)*x.*z + 2*m(6) * y.*z + ...
                  2*m(7) *x    + 2*m(8)*y    + 2*m(9) * z;
        p = patch( isosurface( x, y, z, Ellipsoid, -m(10) ) );
        set( p, 'FaceColor', coltbl(i), 'EdgeColor', 'none' );
        
    end

    view( -70, 40 );
    axis vis3d equal;
    camlight;
    lighting phong;
    alpha(0.4);
end
        