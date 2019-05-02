function isFound = findItem(image, item_name, list, list_threshold)
% Get itemID and return boolean result if found or not

%%  Load the mappings for features and thresholds
% If possible faster to simply call from the environment as here,
% if can't be done then simply uncomment below lines and remove fn args.

%load('featureList.mat', 'list')
%load('thresholdList.mat','list_threshold')

%% Perform matching and assess item apperance
%disp(item_name)

item_name

threshold = list_threshold(item_name);
matches = get_matches(list, image, item_name);

fprintf('\n%i matches\n', size(matches,1));
fprintf('\nThreshold for item "%s" is %i\n',item_name, threshold);

if size(matches,1) >= threshold
    isFound=true;
    fprintf('\nItem found\n');
else
    isFound=false;
    fprintf('\nItem not found\n');
end

%% For easy copy/pasting function testing
%
%       itemlist = {'kong_air_dog_squeakair_tennis_ball',
%                   'laugh_out_loud_joke_book',
%                   'crayola_64_ct',          
%                   'kong_duck_dog_toy',
%                   'kong_sitting_frog_dog_toy',
%                   'elmers_washable_no_run_school_glue',
%                   'adventures_of_huckleberry_finn_book',
%                   'paper_mate_12_count_mirado_black_warrior',
%                   'sharpie_accent_tank_style_highlighters',
%                   'genuine_joe_plastic_stir_sticks'};

%