function [A, b, magB, er]=ellipsoid_fit(XYZ,varargin)
% Fit an (non)rotated ellipsoid or sphere to a set of xyz data points
% XYZ: N(rows) x 3(cols), matrix of N data points (x,y,z)
% optional flag f, default to 0 (fitting of rotated ellipsoid)

A = eye(3);

 x=XYZ(:,1); y=XYZ(:,2); z=XYZ(:,3); 
 if nargin>1
     f=varargin{1};
 else f=0;
 end

 if( f == 5)
[A, b, magB, er, ispd] = correctEllipsoid4(x,y,z);
 end


 if( f == 1)
[A, b, magB, er, ispd] = correctEllipsoid7(x,y,z);
 end

 if( f == 0)
[A, b, magB, er, ispd] = correctEllipsoid10(x,y,z);
 end


end


function [Winv, V, B, er, ispd] = correctEllipsoid4(x,y,z)
% R is the identity

    b = x.*x + y.*y + z.*z;

    A = [x,y,z];
    A = [A ones(numel(x),1)];

    soln = (A'*A)^(-1)*A'*b;
    Winv = eye(3);
    V = 0.5*soln(1:3);
    B = sqrt(soln(4) + sum(V.*V));
    
%      res = residual(Winv, V, B,  [x,y,z])
%       er = (1/(2*B*B))*sqrt(res.'*res/numel(x));
    if nargout > 3
        res = A*soln - b;
        er = (1/(2*B*B) * sqrt( res.'*res / numel(x)));
        ispd = 1;
    else
        er = -ones(1);
        ispd = -1;
    end
end




function [Winv, V, B, er, ispd] = correctEllipsoid7(x,y,z)

    d =  [x.*x,  y.*y,  z.*z, x, y, z, ones(size(x))];

    dtd = d.' * d;

    [evc, evlmtx] = eig(dtd);

    eigvals = diag(evlmtx);
    [~, idx] = min(eigvals);

    beta = evc(:,idx); %solution has smallest eigenvalue
    A = diag(beta(1:3));
    dA = det(A);

    if dA < 0
        A = -A;
        beta = -beta;
        dA = -dA; %Compensate for -A.
    end
    V = -0.5*(beta(4:6)./beta(1:3)); %hard iron offset

    B = sqrt(abs(sum([...
        A(1,1)*V(1)*V(1), ...
        A(2,2)*V(2)*V(2), ...
        A(3,3)*V(3)*V(3), ...
        -beta(end)] ...
    )));
  

    % We correct Winv and B by det(A) because we don't know which has the
    % gain. By convention, normalize A.

    det3root = nthroot(dA,3);
    det6root = sqrt(det3root);
    Winv = sqrtm(A./det3root);
    B = B./det6root;
    
    if nargout > 3
        res = residual(Winv,V,B, [x,y,z]);
        er = (1/(2*B*B))*sqrt(res.'*res/numel(x));
        [~,p] = chol(A);
        ispd = (p == 0);
    else
        er = -ones(1, 'like',x);
        ispd = -1;
    end

    
    
end


function [Winv, V,B,er, ispd] = correctEllipsoid10(x,y,z)

    d = [...
        x.*x, ...
        2*x.*y, ...
        2*x.*z, ...
        y.*y, ...
        2*y.*z, ...
        z.*z, ...
        x, ...
        y, ...
        z, ...
        ones(size(x))];

    dtd = d.' * d;

    [evc, evlmtx] = eig(dtd);

    eigvals = diag(evlmtx);
    [~, idx] = min(eigvals);

    beta = evc(:,idx); %solution has smallest eigenvalue

    A = beta([1 2 3; 2 4 5; 3 5 6]); %make symmetric
    dA = det(A);

    if dA < 0
        A = -A;
        beta = -beta;
        dA = -dA; %Compensate for -A.
    end

    V = -0.5*(A^(-1)*beta(7:9)); %hard iron offset

    B = sqrt(abs(sum([...
        A(1,1)*V(1)*V(1), ...
        2*A(2,1)*V(2)*V(1), ...
        2*A(3,1)*V(3)*V(1), ...
        A(2,2)*V(2)*V(2), ...
        2*A(3,2)*V(2)*V(3), ...
        A(3,3)*V(3)*V(3), ...
        -beta(end)] ...
    )));
  
    % We correct Winv and B by det(A) because we don't know which has the
    % gain. By convention, normalize A.

    det3root = nthroot(dA,3);
    det6root = sqrt(det3root);
    Winv = sqrtm(A./det3root);
    B = B./det6root;
    
    if nargout > 3 
        res = residual(Winv,V,B, [x,y,z]);
        er = (1/(2*B*B))*sqrt(res.'*res/numel(x));
        [~,p] = chol(A);
        ispd = (p == 0);
    else
        er = -ones(1, 'like',x);
        ispd = -1;
    end

end


function r = residual(Winv, V, B, data)
% Residual error after correction

spherept = (Winv * (data.' - V)).'; % a point on the unit sphere
radsq = sum(spherept.^2,2);

r = radsq - B.^2;
end

