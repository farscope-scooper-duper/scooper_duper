function roughSURF(itemImg, sceneImg)
itemPoints = detectSURFFeatures(itemImg);
scenePoints = detectSURFFeatures(sceneImg);

[itemFeatures, itemPoints] = extractFeatures(itemImg, itemPoints);
[sceneFeatures, scenePoints] = extractFeatures(sceneImg, scenePoints);

itemPairs = matchFeatures(itemFeatures, sceneFeatures);

matchedItemPoints = itemPoints(itemPairs(:, 1), :);
matchedScenePoints = scenePoints(itemPairs(:, 2), :);

figure;
showMatchedFeatures(itemImg, sceneImg, matchedItemPoints, ...
    matchedScenePoints, 'montage');
title('Putatively Matched Points (Including Outliers)');

M = mean(matchedScenePoints.Location);
figure;
imshow(sceneImg);
hold on;
plot(M(1), M(2), 'r*');

% [tform, inlierItemPoints, inlierScenePoints] = ...
%     estimateGeometricTransform(matchedItemPoints, matchedScenePoints, 'affine');
% 
% itemPolygon = [1, 1;...                           % top-left
%         size(itemImg, 2), 1;...                 % top-right
%         size(itemImg, 2), size(itemImg, 1);... % bottom-right
%         1, size(itemImg, 1);...                 % bottom-left
%         1, 1];                   % top-left again to close the polygon
% 
% newItemPolygon = transformPointsForward(tform, itemPolygon);
%     
% figure;
% imshow(sceneImg);
% hold on;
% line(newItemPolygon(:, 1), newItemPolygon(:, 2), 'Color', 'y');
% title('Detected Item');
end