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
load('featureList.mat', 'list')
load('thresholdList.mat','list_threshold')
toc

%Subscribes to the item to search for
%Publishes whether item is in view

masterHost = 'localhost';
node_1 = robotics.ros.Node('node_1', masterHost);
node_2 = robotics.ros.Node('node_2', masterHost);
node_3 = robotics.ros.Node('node_3', masterHost);
node_4 = robotics.ros.Node('node_4', masterHost);

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


%test spoofer
%send(itemSpoof,msg);
%

%% While loop to take picture and find item
% Note: need callback function to update for new /target_item
preview(cam)

while true
    tic
    % Get image and search for target_item
    image = snapshot(cam);    
    result.Data = findItem(image, target_item, list, list_threshold);    
    
    %Publish result
    send(itemPub,result);
    toc
end
