originalI = imread('amazonwarehouseNew.png');
% Create a nonflat ball-shaped structuring element.
se = offsetstrel('ball',7,7);
dilatedI = imerode(originalI,se);
imshowpair(originalI,dilatedI,'montage')
imwrite(dilatedI,'amazonwarehouseNewEroded.png')
