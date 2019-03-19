#Potential future issues list:
# - It's currently assumed that there is only ever one instance of each item. There may be undefined behaviour if this isn't the case.
# - This would be worse still if there were two of the same item in the same bin, as the items are stored in lists, not sets.
# - There's no reordering of the data structure from the .json, so it might be inefficient/annoying to work with.
# - It's going to give a grumble in the unlikely case that there's an attempt to add an item to a bin that had nothing in to start with (or rather, that didn't exist in the .json).
# - No assumptions are made about what is a target item and what isn't or how it might change when things are moved.
# - (This is a design quirk) Failing in the sense of dropping the item on the floor is a success to the work order.

#Needs to be able to:
#`Open and read a .json file
#`Query the data:
#` - What is in bin X?
#` - What bin is Y in?
#` - List target items

#Later, we will want to:
#`Add items to bins (including making a tote/floor entry)
#`Remove items from bins
#`Write to a new .json file

#Later still, we will want to:
#`Keep a priority queue of target items
#`Manage the queue so that items are added/removed/pushed back
#Curate the queue according to some measure of pickability

import json

class WorldModel:
    '''Objects of this class represent some model of the environment - namely bins and the items expected to be within them. These are initially drawn from the provided .json file.'''

    def __init__(self, input_file_name): #No checks for if this file exists or not
        with open(input_file_name) as input_file: #This is currently read only
            self.json_data = json.load(input_file)

        self.work_order = self.json_data['work_order'] 
        self.pick_list = self.target_items() #As read, currently, later may be ordered by pickability.
        self.bin_contents = self.json_data['bin_contents']

        #If the pick file doesn't have any items in the tote (or on the floor), we have to make these ourselves.
        self.bin_contents.setdefault('tote', [])
        self.bin_contents.setdefault('floor', [])

    def items_in(self, bin_reference):
        '''Return a list of the items in the bin with the given reference.'''
        return self.bin_contents[bin_reference]

    def bins_of(self, item_reference):
        '''Return the bins in which the item referenced can be found.'''
        return [b for b in self.bin_contents.keys() if item_reference in self.bin_contents[b]]

    #Possibly deprecated
    def target_items(self):
        '''Provides a full list of the target items.'''
        return [i['item'] for i in self.work_order]

    def add_item_to_bin(self, item_ref, bin_ref):
        '''Adds the specified item to the given bin.'''
        self.bin_contents[bin_ref].append(item_ref)

    def remove_item_from_bin(self, item_ref, bin_ref):
        '''Removes one instance of the specified item from the given bin if it exists.'''
        if item_ref in self.bin_contents[bin_ref]:
            self.bin_contents[bin_ref].remove(item_ref)

    def output_to_file(self, output_file_name):
        '''Writes the current state to the given output file.'''
        json_data = {'work_order': self.work_order, 'bin_contents': self.bin_contents}
        with open(output_file_name, 'w') as output_file:
            json.dump(json_data, output_file, sort_keys=True, indent=4)

    def update_work_order(self):
        '''Updates the work order with the currently-believed position for each of the items.'''
        self.work_order = [{'bin':self.bins_of(i), 'item':i} for i in self.pick_list]

    def pick_success(self, item_ref):
        ''' '''
        if item_ref in self.pick_list:
            self.pick_list.remove(item_ref)
            self.update_work_order()

    def pick_failure(self, item_ref):
        '''The item referenced is removed from its position in the work order and moved to the back of the work order list.'''
        if item_ref in self.pick_list:
            self.pick_list.remove(item_ref)
            self.pick_list.append(item_ref)
            self.update_work_order()
        
