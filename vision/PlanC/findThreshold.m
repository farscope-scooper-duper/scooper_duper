function [features] = findThreshold
clear;
cam = webcam(2);
preview(cam);
%cnt = 0;
load ('featureList_all.mat','list')
features = [];

for i = 1:50
    %preview(cam)
    image = snapshot(cam);
    item_name = 'feline_greenies_dental_treats';
    feat = getMatchesTest(list, image, item_name);
    
    features = [features;feat];

    %features[cnt] = feat;
    pause(1)
    %cnt=cnt + 1;
end
end