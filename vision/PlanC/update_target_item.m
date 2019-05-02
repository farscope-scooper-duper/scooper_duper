function update_target_item(~,message)
global items;
global targ;
target_item = message.Data;

items = strsplit(target_item,',');

targ=items{1};
items(1)=[];
end
