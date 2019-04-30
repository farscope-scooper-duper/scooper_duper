function [points,features] = SURFextractfeatures_gray(image)
image = rgb2gray(image);
points = detectSURFFeatures(image);
points= points(points.Metric<1500000); %filter
[features, points] = extractFeatures(image, points);
end

