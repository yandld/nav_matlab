r=imread('reef.jpg','jpg');
%mage(r)

rb= r(:,:,3);
imagesc(rb);
colormap(gray);

rbt=rb<130; %threshold dark values
imagesc(rbt)
colormap(gray)


% calculate sum of all white points (=1)
reef=sum(sum(rbt)); %reef pts
totalpts=prod(size(rbt)); %#pts in image
percentreef=reef/totalpts
