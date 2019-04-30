function [matches] = get_matches(list,image,item_name)

%Process scene image
image = rgb2gray(image);
scene_points = detectSURFFeatures(image);
%scene_points= scene_points(scene_points.Metric<1500); %filter
[scene_features, scene_points] = extractFeatures(image, scene_points);

%Get reference features for desired item
SURF_target_item = list(item_name);
ref_points= SURF_target_item{1};
ref_features= SURF_target_item{2};

%Match features
pairs = matchFeatures(ref_features, scene_features);
ref_matched   = ref_points(pairs(:,1),:);
scene_matched = scene_points(pairs(:,2),:);

%output
matches=scene_matched;

%% Visualisations (left image is not useful in this, ref is several ims)
%figure;
%showMatchedFeatures(image, image, ref_matched, scene_matched, 'montage');
%title('Putatively Matched Points (Including Outliers)');

end


