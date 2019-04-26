function colSURF(objIm,sceneIm)
close all;

%Container
matchPointLocations =[]; 

for i=1:3 %Channels RGB
    %Get SURF points
    objImC = objIm(:,:,i);
    sceneImC = sceneIm(:,:,i);
    objPoints = detectSURFFeatures(objImC);
    scenePoints = detectSURFFeatures(sceneImC);
    [objFeatures,objPoints] = extractFeatures(objImC,objPoints);
    [sceneFeatures,scenePoints] = extractFeatures(sceneImC,scenePoints);
    
    %Find matching
    pairs = matchFeatures(objFeatures, sceneFeatures);
    objMatched   =   objPoints(pairs(:,1),:);
    sceneMatched = scenePoints(pairs(:,2),:);
    sceneMatched.Metric
    objMatched = selectStrongest(objMatched, sceneMatched.Count);
    %sceneMatchedStrongest = selectStrongest(sceneMatched, 5)
    matchPointLocations = vertcat(matchPointLocations, sceneMatched.Location);
end

%Outputs
matchPointLocations=unique(floor(matchPointLocations),'rows'); %cut dupes
size(matchPointLocations,1)
centrePoint = floor(mean(matchPointLocations));

%Visualisations
figure;
showMatchedFeatures(objIm, sceneIm, objMatched, sceneMatched, 'montage');
title('Putatively Matched Points (Including Outliers)');

figure;
imshow(sceneIm);
hold on;
plot(centrePoint(1),centrePoint(2),'*y');

end

