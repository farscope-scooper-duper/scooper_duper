
% Publish 1 ---> item was identified 
% Publish 0 ----> item not identified


masterHost = 'localhost';
node_1 = robotics.ros.Node('node_1', masterHost);
node_2 = robotics.ros.Node('node_2', masterHost);
node_3 = robotics.ros.Node('node_3', masterHost);


% Create a publisher and subscriber for the '/item_found' topic
itemPub = robotics.ros.Publisher(node_1,'/item_found','std_msgs/Bool');
itemPubmsg = rosmessage(itemPub);
itemSub = robotics.ros.Subscriber(node_2,'/item_found');


% Create a subscriber for the '/scan' topic --> itemID
scanSub = robotics.ros.Subscriber(node_3,'/scan');

scansub = rossubscriber('/scan');

itemPub = findItem(scansub);

% Create a timer for publishing messages and assign appropriate handles
% The timer will call exampleHelperROSSimTimer at a rate of 10 Hz.
timerHandles.twistPub = itemPub;
timerHandles.twistPubmsg = itemPubmsg;
timerHandles.scanPub = scanPub;
timerHandles.scanPubmsg = scanPubmsg;
simTimer = ExampleHelperROSTimer(0.1, {@exampleHelperROSSimTimer,timerHandles});