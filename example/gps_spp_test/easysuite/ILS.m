function [afixed,sqnorm]=ILS(ahat,Qahat,ncands)
%
%   [afixed,sqnorm]=ILS(ahat,Qahat,ncands)
%
% This routine allows to apply Integer Least Squares estimation using the
% LAMBDA method. In contrast to the main LAMBDA routine, in which different
% methods can be chosen, this routine by default applies ILS with the
% search-and-shrink procedure. Furthermore, no intermediate output is
% generated or stored in order to save CPU time. All the subroutines called
% by LAMBDA are integrated in this routine (and hence are not called from
% this program).
% If a user is interested in for example the Z-matrix or success rate, it
% is better to use LAMBDA.m
%
% INPUTS:
%
%     ahat: Float ambiguities (must be a column!)
%    Qahat: Variance/covariance matrix of ambiguities 
%   ncands: number of integer candidate vectors       
%
% OUTPUTS:
%
%   afixed: Array of size (n x ncands) with the estimated integer
%           candidates, sorted according to the corresponding squared norms, 
%           best candidate first. 
%   sqnorm: Distance between integer candidate and float ambiguity vectors 
%           in the metric of the variance-covariance matrix Qahat.
%
%
%
%------------------------------------------------------------------
% DATE    : 11-APRIL-2012                                          
% Authors : Sandra VERHAGEN                                             
%           GNSS Research Center, Curtin University
%           Mathematical Geodesy and Positioning, Delft University of
%           Technology                              
%------------------------------------------------------------------
%
% REFERENCES: 
%  1. LAMBDA Software Package: Matlab implementation, Version 3.0.
%     Documentation provided with the software.
%  2. de Jonge P, Tiberius C (1996) The LAMBDA method of intger ambiguity 
%     estimation:implementation aspects.
%  3. Chang X ,Yang X, Zhou T (2005) MLAMBDA: a modified LAMBDA method for
%     integer least-squares estimation
%  4. Teunissen P (1993) Least-squares estimation of the integer GPS
%     ambiguities. In: Invited lecture, section IV theory and methodology,
%     IAG General Meeting, Beijing, China
%  5. Teunissen P (1995) The least-squares ambiguity decorrelation
%     adjustment: a method for fast GPS ambiguity estitmation. J Geod
%     70:651-7

%=============================START PROGRAM===============================%

if(nargin<2)
    error(['Not enough inputs: float solution',  ...
        'and its variance-covariance matrix must be specified']);
end

n      = size (Qahat,1);
afixed = zeros(n,ncands);
sqnorm = zeros(1,ncands);

%-----------------------------------------------------------------
%Tests on Inputs ahat and Qahat                           

%Is the Q-matrix symmetric?
if ~isequal(Qahat-Qahat'<1E-8,ones(size(Qahat)));
  error ('Variance-covariance matrix is not symmetric!');
end;

%Is the Q-matrix positive-definite?
if sum(eig(Qahat)>0) ~= size(Qahat,1);
  error ('Variance-covariance matrix is not positive definite!');
end;

%Do the Q-matrix and ambiguity-vector have identical dimensions?
if length(ahat) ~= size(Qahat,1);
  error (['Variance-covariance matrix and vector of ambiguities do', ...
      'not have identical dimensions!']);
end;

%Is the ambiguity vector a column?  
if size(ahat,2) ~= 1;
  error ('Ambiguity-vector should be a column-vector');
end;


%remove integer numbers from float solution, so that all values are between
%-1 and 1 (for computational convenience only)
incr = ahat - rem(ahat,1);
ahat = rem(ahat,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DECORRELATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Compute Z matrix based on the decomposition  Q=L^T*D*L; The transformed
%float solution: \hat{a} = Z^T *ahat, Qzhat = Z^T * Qahat * Z

iZt  = eye(n);
i1   = n - 1;
sw   = 1;

% --------------------------
% --- LtDL decomposition ---
% --------------------------

[L,D] = ldldecom (Qahat);

% ------------------------------------------
% --- The actual decorrelation procedure ---
% ------------------------------------------

while sw;

   i  = n;   %loop for column from n to 1
   sw = 0;

   while ( ~sw ) && (i > 1)

      i = i - 1;  %the ith column
      if (i <= i1); 
      
         for j = i+1:n
            mu = round(L(j,i));
            if mu ~= 0
               L(j:n,i) = L(j:n,i) - mu * L(j:n,j);
               %iZt is Z (transposed inverse) matrix 
               iZt(1:n,j) = iZt(1:n,j) + mu * iZt(1:n,i);  
            end
         end

      end;

      delta = D(i) + L(i+1,i)^2 * D(i+1);
      if (delta < D(i+1))

         lambda(3)    = D(i+1) * L(i+1,i) / delta;
         eta          = D(i) / delta;
         D(i)         = eta * D(i+1);
         D(i+1)       = delta;

         Help         = L(i+1,1:i-1) - L(i+1,i) .* L(i,1:i-1);
         L(i+1,1:i-1) = lambda(3) * L(i+1,1:i-1) + eta * L(i,1:i-1);
         L(i,1:i-1)   = Help;
         L(i+1,i)     = lambda(3);

         % swap rows i and i+1
         L(i+2:n,i:i+1) = L(i+2:n,i+1:-1:i);
         iZt(:,i:i+1) = iZt(:,i+1:-1:i);

         i1           = i;
         sw           = 1;

      end;

   end;

end;

% decorrelated ambiguities
ahat = iZt\ahat;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SEARCH
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
afixed = zeros(n, ncands);
sqnorm = zeros(1, ncands);

%initializing the variables for searching
Chi2     = 1.0e+18;        %start search with an infinite chi^2
dist(n)  = 0;              %dist(k)=sum_{j=k+1}^{n}(a_j-acond_j)^2/d_j 
endsearch= false;
count    = 0;              %the number of candidates

acond(n) = ahat(n);
zcond(n) = round(acond(n));
left     = acond(n) - zcond(n);
step(n)  = sign(left);

%-----------------------------------------------------------------------%
%For a very occasional case when the value of float solution ahat(n)==0, we
%compusively give a positive step to continue. This case can
%actually never happen in reality, but only when the exact integer value
%is specified for ahat. 
if step(n)==0
    step(n) = 1;
end
%------------------------------------------------------------------------%

imax     = ncands;         %initially, the maximum F(z) is at ncands

S(1:n, 1:n) = 0;           %used to compute conditional ambiguities

k = n;

%Start the main search-loop
while ~ (endsearch);
    %newdist=sum_{j=k}^{n}(a_j-acond_j)^2/d_j=dist(k)+(a_k-acond_k)^2/d_k
    
    newdist = dist(k) + left^2/D(k);
    
    if (newdist < Chi2)
        
        if (k~=1)         %Case 1: move down
            k = k - 1;
            dist(k)  = newdist;
            S(k,1:k) = S(k+1,1:k) +(zcond(k+1)-acond(k+1))*L(k+1,1:k);
            
            acond(k) = ahat(k) + S(k, k);
            zcond(k) = round(acond(k));
            left     = acond(k) - zcond(k);
            step(k)  = sign(left);
            
            %-----------------------------------------------------------------------%
            %For a very occasional case when the value of float solution ahat(n)==0, we
            %compusively give a positive step to continue. This case can
            %actually never happen in reality, but only when the exact integer value
            %is specified for ahat. 
            if (step(k)==0),  step(k) = 1;  end
            %------------------------------------------------------------------------%
        else
            
            %Case 2: store the found candidate and try next valid integer
            if (count < ncands - 1) 
                %store the first ncands-1 initial points as candidates
                
                count = count + 1;
                afixed(:, count) = zcond(1:n);
                sqnorm(count) = newdist;          %store f(zcond)
           
            else
                
                afixed(:,imax) = zcond(1:n);
                sqnorm(imax)   = newdist;
                [Chi2, imax]   = max(sqnorm);
                
            end
            
            zcond(1) = zcond(1) + step(1);     %next valid integer
            left     = acond(1) - zcond(1);
            step(1)  =-step(1)  - sign(step(1)); 
        end
        
    else
        %Case 3: exit or move up
        if (k == n)
            endsearch = true;
        else 
            k        = k + 1;         %move up
            zcond(k) = zcond(k) + step(k);  %next valid integer
            left     = acond(k) - zcond(k);
            step(k)  =-step(k)  - sign(step(k));
        end
    end
end

[sqnorm, order]=sort(sqnorm);
afixed = afixed(:,order);

%Perform the back-transformation and add the increments
afixed = iZt*afixed; 
afixed = afixed + repmat(incr,1,ncands);

return;
