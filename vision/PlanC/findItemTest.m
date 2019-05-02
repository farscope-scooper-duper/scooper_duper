function [pairfeat,isFound] = findItemTest(image, item_name, list, list_threshold)
% Get itemID and return boolean result if found or not

%%  Load the mappings for features and thresholds
% If possible faster to simply call from the environment as here,
% if can't be done then simply uncomment below lines and remove fn args.

%load('featureList.mat', 'list')
%load('thresholdList.mat','list_threshold')

%% Perform matching and assess item apperance
threshold = list_threshold(item_name);
matches = get_matches(list, image, item_name);

pairfeat = size(matches,1);


item_name

fprintf('\n%i matches\n', size(matches,1));
fprintf('\nThreshold for item "%s" is %i\n',item_name, threshold);

if size(matches,1) >= threshold
    isFound=1;
    fprintf('\nItem found\n');
else
    isFound=0;
    fprintf('\nItem not found\n');
end
