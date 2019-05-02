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
global target_item;
target_item = 'elmers_washable_no_run_school_glue';
rossubscriber('/target_item', @update_target_item);
items = strsplit(target_item,',');

targ=items(1);
items(1)=[];

%% While loop to take picture and find item
% Note: need callback function to update for new /target_item
preview(cam)
result.Data = false;
while true
    tic
    % Get image and search for target_item
    image = snapshot(cam);    
    positive = findItem(image, targ, list, list_threshold);    
    
    %Check for false positives
    if positive ==true
        false_postive=false; %init
        i=0;
        fprintf('Searching for false positives...');
        while(false_positive==false || i<length(items))
            false_positive = findItem(image, items(i), list, list_threshold);
            i=i+1;
        end
        if false_postive ==true
            result.Data =false;
            fprintf('False postive found: %s',items(i));
            fprintf('Result rejected');
        else
            result.Data =true;
        end
    end

    %Publish result
    send(itemPub,result);
    
    toc
end
