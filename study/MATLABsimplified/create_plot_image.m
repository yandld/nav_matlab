rows=20; % variable for # rows
stripes=5;% variable for # stripes

% make one column of 1's and an adjacent column of 0's
x=horzcat( zeros(rows,1), ones(rows,1), ones(rows,1)*2, ones(rows,1)*3,ones(rows,1)*4);

y=[]; %initialize and clear y matrix
%for n=1:stripes
%y=horzcat(y,x);% concatenate x onto y
%end

y = x;
clim=[0 4]; % color limits for imagesc
imagesc(y, clim); % plot scaled image
colormap(gray); % use gray scale color map