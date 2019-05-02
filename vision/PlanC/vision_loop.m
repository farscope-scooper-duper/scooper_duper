%% ROS integration for vision
% input: /item_in_view
% output: /target_item
rosshutdown;
clc;
clear all;
rosinit;

% Init camera
cam = webcam(1);

% load in lists
tic
load('featureList_all.mat', 'list')
load('thresholdList_all.mat','list_threshold')

%load('featureList.mat', 'list')%JUJU
%load('thresholdList.mat','list_threshold')
toc

%Subscribes to the item to search for
%Publishes whether item is in view

masterHost = 'localhost';
node_1 = robotics.ros.Node('node_1', masterHost);


%%
% Create a publisher and subscriber for the '/item_found' topic
itemPub = robotics.ros.Publisher(node_1,'/item_in_view','std_msgs/Bool');
itemPubmsg = rosmessage(itemPub);
result=rosmessage('std_msgs/Bool');

%spoofer
%itemSpoof= robotics.ros.Publisher(node_3, '/target_item','std_msgs/String');
%msg = rosmessage('std_msgs/String');
%msg.Data = 'laugh_out_loud_joke_book';
%
% 
global targ;
global items;
targ = 'elmers_washable_no_run_school_glue';
items = [];

rossubscriber('/target_item', @update_target_item);


%% While loop to take picture and find item
% Note: need callback function to update for new /target_item
preview(cam)

while true
    tic
    result.Data = false;
    % Get image and search for target_item
    image = snapshot(cam);    
    positive = findItem(image, targ, list, list_threshold);    
    
    %Check for false positives
    if positive == true
        false_positive = false; %init
        fprintf('Searching for false positives...');
        
        for i = items
           if (false_positive == false)
              false_positive = findItem(image, i{1}, list, list_threshold); 
           end
        end
        if (false_positive == true && list_threshold(i{1})>30) 
            result.Data = false;
            %fprintf('False positive found: %s',items(i));
            %fprintf('Result rejected');
        else
            result.Data = true;
        end
    end

    %Publish result
    send(itemPub,result);
    
    toc
end
