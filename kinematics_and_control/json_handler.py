#!/usr/bin/env python
#Potential future issues list:
# - It's currently assumed that there is only ever one instance of each item. There may be undefined behaviour if this isn't the case.
# - This would be worse still if there were two of the same item in the same bin, as the items are stored in lists, not sets.
# - There's no reordering of the data structure from the .json, so it might be inefficient/annoying to work with.
# - It's going to give a grumble in the unlikely case that there's an attempt to add an item to a bin that had nothing in to start with (or rather, that didn't exist in the .json).
# - No assumptions are made about what is a target item and what isn't or how it might change when things are moved.
# - (This is a design quirk) Failing in the sense of dropping the item on the floor is a success to the work order.
# - Update work order first thing.

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
#`Curate the queue according to some measure of pickability


import json

class WorldModel:
    '''Objects of this class represent some model of the environment - namely bins and the items expected to be within them. These are initially drawn from the provided .json file.'''

    def __init__(self, input_file_name): #No checks for if this file exists or not
        with open(input_file_name) as input_file: #This is currently read only
            self.json_data = json.load(input_file)

        self.work_order = self.json_data['work_order'] 
        self.failed_picks = []
        self.pick_list = self.target_items() #Read in first
        self.bin_contents = self.json_data['bin_contents']

        #If the pick file doesn't have any items in the tote (or on the floor), we have to make these ourselves.
        self.bin_contents.setdefault('tote', [])
        self.bin_contents.setdefault('floor', [])
    
        self.verbosity = True

        self.strategise_picks()

    def strategise_picks(self):
        #Pick list order is: lonely items by pickability; crowded items by crowdedness then feature; failed picks
        lonely_picks = []
        crowded_picks = []
        if (len(self.failed_picks) == len(self.pick_list)): #If every item has been failed
            self.failed_picks = [] #We can pick any item again

        for item in self.pick_list:
            if (item in self.failed_picks):
                pass
            elif (len(self.items_in(self.bins_of(item)[0])) == 1): 
                lonely_picks.append(item)
            else:
                crowded_picks.append(item)

        lonely_picks.sort(key=self.pickability)
        crowded_picks.sort(key= lambda i: (len(self.items_in(self.bins_of(i)[0])), self.features(i)))

        self.pick_list = lonely_picks + crowded_picks + self.failed_picks    


    def pickability(self, item): #The higher the better
        item_pickabilities = { 
            "kong_sitting_frog_dog_toy"                 : 1,
            "kong_duck_dog_toy"                         : 1,
            "feline_greenies_dental_treats"             : 1,
            "crayola_64_ct"                             : 1,
            "genuine_joe_plastic_stir_sticks"           : 1,
            "mommys_helper_outlet_plugs"                : 1,
            "kong_air_dog_squeakair_tennis_ball"        : 1,
            "dove_beauty_bar"                           : 1,
            "dr_browns_bottle_brush"                    : 1,
            "one_with_nature_soap_dead_sea_mud"         : 1,
            "first_years_take_and_toss_straw_cup"       : 1,
            "safety_works_safety_glasses"               : 1,
            "mark_twain_huckleberry_finn"               : 1,
            "laugh_out_loud_joke_book"                  : 1,
            "mead_index_cards"                          : 1,
            "kyjen_squeakin_eggs_plush_puppies"         : 1,
            "expo_dry_erase_board_eraser"               : 1,
            "sharpie_accent_tank_style_highlighters"    : 1,
            "paper_mate_12_count_mirado_black_warrior"  : 1,
            "munchkin_white_hot_duck_bath_toy"          : 1,
            "highland_6539_self_stick_notes"            : 1,
            "elmers_washable_no_run_school_glue"        : 1,
            "champion_copper_plus_spark_plug"           : 1}
        return(item_pickabilities.get(item, 0))    

    def features(self, item):
        item_features = { 
            "kong_sitting_frog_dog_toy"                 : 1,
            "kong_duck_dog_toy"                         : 1,
            "feline_greenies_dental_treats"             : 1,
            "crayola_64_ct"                             : 1,
            "genuine_joe_plastic_stir_sticks"           : 1,
            "mommys_helper_outlet_plugs"                : 1,
            "kong_air_dog_squeakair_tennis_ball"        : 1,
            "dove_beauty_bar"                           : 1,
            "dr_browns_bottle_brush"                    : 1,
            "one_with_nature_soap_dead_sea_mud"         : 1,
            "first_years_take_and_toss_straw_cup"       : 1,
            "safety_works_safety_glasses"               : 1,
            "mark_twain_huckleberry_finn"               : 1,
            "laugh_out_loud_joke_book"                  : 1,
            "mead_index_cards"                          : 1,
            "kyjen_squeakin_eggs_plush_puppies"         : 1,
            "expo_dry_erase_board_eraser"               : 1,
            "sharpie_accent_tank_style_highlighters"    : 1,
            "paper_mate_12_count_mirado_black_warrior"  : 1,
            "munchkin_white_hot_duck_bath_toy"          : 1,
            "highland_6539_self_stick_notes"            : 1,
            "elmers_washable_no_run_school_glue"        : 1,
            "champion_copper_plus_spark_plug"           : 1}
        return(item_features.get(item, 0))

    def set_verbosity(self, v):
        '''Sets whether the world model should output its operations as it goes.'''        
        self.verbosity = v

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
        if (self.verbosity == True):
            print("Added %s to %s." % (item_ref, bin_ref))

    def remove_item_from_bin(self, item_ref, bin_ref):
        '''Removes one instance of the specified item from the given bin if it exists.'''
        if item_ref in self.bin_contents[bin_ref]:
            self.bin_contents[bin_ref].remove(item_ref)
            if (self.verbosity == True):
                print("Removed %s from %s." % (item_ref, bin_ref))

    def output_to_file(self, output_file_name):
        '''Writes the current state to the given output file.'''
        json_data = {'work_order': self.work_order, 'bin_contents': self.bin_contents}
        with open(output_file_name, 'w') as output_file:
            json.dump(json_data, output_file, sort_keys=True, indent=4)
            if (self.verbosity == True):
                print("Output pick data to %s." % output_file_name)

    def update_work_order(self):
        '''Updates the work order with the currently-believed position for each of the items.'''
        if (self.verbosity == True):
            print("Updated the work order.")
        self.strategise_picks()
        self.work_order = [{'bin':self.bins_of(i), 'item':i} for i in self.pick_list]

    def pick_success(self, item_ref):
        '''The item referenced is removed from the work order '''
        if item_ref in self.pick_list:
            if (self.verbosity == True):
                print("Pick success: removed %s from the pick queue." % item_ref)
            self.pick_list.remove(item_ref)
            self.update_work_order()

    def pick_failure(self, item_ref):
        '''The item referenced is removed from its position in the work order and moved to the back of the work order list.'''
        if item_ref in self.pick_list:
            if (self.verbosity == True):
                print("Pick failure: moved %s to the back of the pick queue." % item_ref)
            self.failed_picks.append(item_ref)
            self.update_work_order()
        
