function [centroid] = get_centroid(matches)
% Gets the centroid pixel of a set of SURFpoint matches
% returns vector in (x,y) between [-1,1] dependant on dist to edge (=1)
% i.e. (0,0) represents the centre of the image.

%Get match locations and find centre pixels
match_locations = matches.Location;
centroid_raw = round(mean(match_locations));

% Fix offset & scaling
offset= [1280/2,960/2]; % resolution dependant
centroid = centroid_raw - offset;
centroid = [centroid(1)/640, - centroid(2)/480];
end

