%% CreateList
% creates the list of object features linked to name and threshold, to be
% ran once if updating item list or thresholds.
clc;

load refIMs.mat

%open featureList.mat

%itemList = cell(1,6,length('refIms.mat'));
      
      
     %% balls object
      squeak_back = imresize(imread('squeak_back.jpg'),0.3);
      squeak_front = imresize(imread('squeak_front.jpg'),0.3);
  
      [points1, features1] = SURFextractfeatures_gray(squeak_back);
      [points2, features2] = SURFextractfeatures_gray(squeak_front);
      balls_points = [points1;points2];
      balls_features = [features1;features2];
      
      balls_SURF_g = {balls_points, balls_features};
      
      %% book object
      jokebook_rear = imresize(imread('jokebook_rear.jpg'),0.3);
      [points1, features1] = SURFextractfeatures_gray(jokebook_rear);
      [points2, features2] = SURFextractfeatures_gray(book_front);
      all_points = [points1;points2];
      all_features = [features1;features2];
      
      book_SURF_g = {all_points, all_features};
      
      %% Crayon
      
      [points1, features1] = SURFextractfeatures_gray(crayon_back);
      [points2, features2] = SURFextractfeatures_gray(crayon_bottom);
      [points3, features3] = SURFextractfeatures_gray(crayon_front);
      [points4, features4] = SURFextractfeatures_gray(crayon_side1);
      [points5, features5] = SURFextractfeatures_gray(crayon_side2);
      [points6, features6] = SURFextractfeatures_gray(crayon_top);
      
      
      all_points = [points1;points2;points3;points4;points5;points6];
      all_features = [features1;features2;features3;features4;features5;features6];
      
      crayon_SURF_g = {all_points, all_features};
      
      
      %% duck
      duck_front2= imresize(imread('duck_front2.jpg'),0.3);
      [points1, features1] = SURFextractfeatures_gray(duck_back);
      [points2, features2] = SURFextractfeatures_gray(duck_front);
      [points3, features3] = SURFextractfeatures_gray(duck_front2);
      all_points = [points1;points2;points3];
      all_features = [features1;features2;features3];
      
      duck_SURF_g = {all_points, all_features};
      
      %% frog
      
      [points1, features1] = SURFextractfeatures_gray(frog_back);
      [points2, features2] = SURFextractfeatures_gray(frog_front);
      all_points = [points1;points2];
      all_features = [features1;features2];
      
      frog_SURF_g = {all_points, all_features};
      
      %% glue
      
      [points1, features1] = SURFextractfeatures_gray(glue_back);
      [points2, features2] = SURFextractfeatures_gray(glue_bottom);
      [points3, features3] = SURFextractfeatures_gray(glue_top);
      all_points = [points1;points2;points3];
      all_features = [features1;features2;features3];
      
      glue_SURF_g = {all_points, all_features};
      
      %% hukbook
      
      [points1, features1] = SURFextractfeatures_gray(hukbook_back);
      [points2, features2] = SURFextractfeatures_gray(hukbook_front);
      [points3, features3] = SURFextractfeatures_gray(hukbook_spine);
      all_points = [points1;points2;points3];
      all_features = [features1;features2;features3];
      
      hukbook_SURF_g = {all_points, all_features};
      
      %% Pencil
      pencils_back= imresize(imread('pencils_back.jpg'),0.3);
      pencils_front= imresize(imread('pencils_front.jpg'),0.3);
      [points1, features1] = SURFextractfeatures_gray(pencils_back);
      [points2, features2] = SURFextractfeatures_gray(pencils_front);
      all_points = [points1;points2];
      all_features = [features1;features2];
      
      pencil_SURF_g = {all_points, all_features};
      
      %% Sharpie
      [points1, features1] = SURFextractfeatures_gray(sharpie_back);
      [points2, features2] = SURFextractfeatures_gray(sharpie_front);
      all_points = [points1;points2];
      all_features = [features1;features2];
      
      sharpie_SURF_g = {all_points, all_features};
      
      %% stir sticks
      
      [points1, features1] = SURFextractfeatures_gray(stir_stick_back);
      [points2, features2] = SURFextractfeatures_gray(stir_stick_front);
      [points3, features3] = SURFextractfeatures_gray(stir_stick_side);
      all_points = [points1;points2;points3];
      all_features = [features1;features2;features3];
      
      stir_stick_SURF_g = {all_points, all_features};
      
      %% catfood
      catfood_back = imresize(imread('catfood_back.jpg'),0.3);
      catfood_front = imresize(imread('catfood_front.jpg'),0.3);    
      [points1, features1] = SURFextractfeatures_gray(catfood_back);
      [points2, features2] = SURFextractfeatures_gray(catfood_front);
      all_points = [points1;points2];
      all_features = [features1;features2];
      
      catfood_SURF_g = {all_points, all_features};
      
      %% self stick notes
      
      self_stick_back = imresize(imread('self_stick_back.jpg'),0.3);
      self_stick_bottom = imresize(imread('self_stick_bottom.jpg'),0.3);
      self_stick_front = imresize(imread('self_stick_front.jpg'),0.3);
      self_stick_top = imresize(imread('self_stick_top.jpg'),0.3);
      [points1, features1] = SURFextractfeatures_gray(self_stick_back);
      [points2, features2] = SURFextractfeatures_gray(self_stick_bottom);
      [points3, features3] = SURFextractfeatures_gray(self_stick_front);
      [points4, features4] = SURFextractfeatures_gray(self_stick_top);
      all_points = [points1;points2;points3;points4];
      all_features = [features1;features2;features3;features4];
      
      self_stick_SURF_g = {all_points, all_features};
      
      
      %% Cups
      cups_back = imresize(imread('cups_back.jpg'),0.3);
      cups_front = imresize(imread('cups_front.jpg'),0.3);
      cups_side= imresize(imread('cups_side.jpg'),0.3);
      cups_side2= imresize(imread('cups_side2.jpg'),0.3);
      [points1, features1] = SURFextractfeatures_gray(cups_back);
      [points2, features2] = SURFextractfeatures_gray(cups_front);
      [points3, features3] = SURFextractfeatures_gray(cups_side);
      [points4, features4] = SURFextractfeatures_gray(cups_side2);
      all_points = [points1;points2;points3;points4];
      all_features = [features1;features2;features3;features4];
      cups_SURF_g = {all_points, all_features};
      
      
      %% Tennis
      tennis_back = imresize(imread('tennis_back.jpg'),0.3);
      tennis_front = imresize(imread('tennis_front.jpg'),0.3);    
      [points1, features1] = SURFextractfeatures_gray(tennis_back);
      [points2, features2] = SURFextractfeatures_gray(tennis_front);
      all_points = [points1;points2];
      all_features = [features1;features2];
      
      tennis_SURF_g = {all_points, all_features};
      
      %% Dove
      dove_bottom = imresize(imread('dove_bottom.jpg'),0.3);
      dove_front = imresize(imread('dove_front.jpg'),0.3);
      dove_side= imresize(imread('dove_side.jpg'),0.3);
      dove_top= imresize(imread('dove_top.jpg'),0.3);
      [points1, features1] = SURFextractfeatures_gray(dove_top);
      [points2, features2] = SURFextractfeatures_gray(dove_front);
      [points3, features3] = SURFextractfeatures_gray(dove_bottom);
      [points4, features4] = SURFextractfeatures_gray(dove_side);
      all_points = [points1;points2;points3;points4];
      all_features = [features1;features2;features3;features4];
      dove_SURF_g = {all_points, all_features};
      
      %% Soap
      soap_front = imresize(imread('soap_front.jpg'),0.3);
      soap_side = imresize(imread('soap_side.jpg'),0.3);
      soap_side2= imresize(imread('soap_side2.jpg'),0.3);
      soap_side3= imresize(imread('soap_side3.jpg'),0.3);
      [points1, features1] = SURFextractfeatures_gray(soap_front);
      [points2, features2] = SURFextractfeatures_gray(soap_side);
      [points3, features3] = SURFextractfeatures_gray(soap_side2);
      [points4, features4] = SURFextractfeatures_gray(soap_side3);
      all_points = [points1;points2;points3;points4];
      all_features = [features1;features2;features3;features4];
      soap_SURF_g = {all_points, all_features};
          
      %% Plug
      plug_back = imresize(imread('plug_back.jpg'),0.3);
      plug_front = imresize(imread('plug_front.jpg'),0.3);    
      [points1, features1] = SURFextractfeatures_gray(plug_back);
      [points2, features2] = SURFextractfeatures_gray(plug_front);
      all_points = [points1;points2];
      all_features = [features1;features2];
      
      plug_SURF_g = {all_points, all_features}; 
      
      %% Brush
      brush_back = imresize(imread('brush_back.jpg'),0.3);
      brush_front = imresize(imread('brush_front.jpg'),0.3);    
      [points1, features1] = SURFextractfeatures_gray(brush_back);
      [points2, features2] = SURFextractfeatures_gray(brush_front);
      all_points = [points1;points2];
      all_features = [features1;features2];
      
      brush_SURF_g = {all_points, all_features}; 
      
      %% Eraser
      eraser_bottom = imresize(imread('eraser_bottom.jpg'),0.3);
      eraser_front = imresize(imread('eraser_front.jpg'),0.3);
      eraser_side= imresize(imread('eraser_side.jpg'),0.3);
      eraser_side2= imresize(imread('eraser_side2.jpg'),0.3);
      eraser_top= imresize(imread('eraser_top.jpg'),0.3);
        
      [points1, features1] = SURFextractfeatures_gray(eraser_bottom);
      [points2, features2] = SURFextractfeatures_gray(eraser_front);
      [points3, features3] = SURFextractfeatures_gray(eraser_side);
      [points4, features4] = SURFextractfeatures_gray(eraser_side2);
      [points5, features5] = SURFextractfeatures_gray(eraser_top);
      
      all_points = [points1;points2;points3;points4;points5];
      all_features = [features1;features2;features3;features4;features5];
      eraser_SURF_g = {all_points, all_features};
      
      
      %% Glasses
      glasses_back = imresize(imread('glasses_back.jpg'),0.3);
      glasses_front = imresize(imread('glasses_front.jpg'),0.3);
      glasses_front2 = imresize(imread('glasses_front2.jpg'),0.3);   
      
      [points1, features1] = SURFextractfeatures_gray(glasses_back);
      [points2, features2] = SURFextractfeatures_gray(glasses_front);
      [points3, features3] = SURFextractfeatures_gray(glasses_front2);
      all_points = [points1;points2;points3];
      all_features = [features1;features2;features3];
      
      glasses_SURF_g = {all_points, all_features};
      
      %% outlet plugs
      outletplugs_back = imresize(imread('outletplugs_back.jpg'),0.3);
      outletplugs_detail = imresize(imread('outletplugs_detail.jpg'),0.3);
      outletplugs_front= imresize(imread('outletplugs_front.jpg'),0.3);   
      
      [points1, features1] = SURFextractfeatures_gray(outletplugs_back);
      [points2, features2] = SURFextractfeatures_gray(outletplugs_detail);
      [points3, features3] = SURFextractfeatures_gray(outletplugs_front);
      all_points = [points1;points2;points3];
      all_features = [features1;features2;features3];
      
      outletplugs_SURF_g = {all_points, all_features};
      
      %% Index (for real)
      
      index_front = imresize(imread('index_front.jpg'),0.3);
      index_top = imresize(imread('index_top.jpg'),0.3);
      [points1, features1] = SURFextractfeatures_gray(index_top);
      [points2, features2] = SURFextractfeatures_gray(index_front);
      all_points = [points1;points2];
      all_features = [features1;features2];
      index_SURF_g = {all_points, all_features};
      
      %% RUBBERduck

      rubberduck_front = imresize(imread('rubberduck_front.jpg'),0.3);
      rubberduck_back= imresize(imread('rubberduck_back.jpg'),0.3);
      
      [points1, features1] = SURFextractfeatures_gray(rubberduck_back);
      [points2, features2] = SURFextractfeatures_gray(rubberduck_front);
      all_points = [points1;points2];
      all_features = [features1;features2];
      
      rubberduck_SURF_g = {all_points, all_features};
      
      

      %% Itemlist
            itemlist = {'kyjen_squeakin_eggs_plush_puppies',
                  'laugh_out_loud_joke_book',
                  'crayola_64_ct',          
                  'kong_duck_dog_toy',
                  'kong_sitting_frog_dog_toy',
                  'elmers_washable_no_run_school_glue',
                  'mark_twain_huckleberry_finn',
                  'paper_mate_12_count_mirado_black_warrior',
                  'sharpie_accent_tank_style_highlighters',
                  'genuine_joe_plastic_stir_sticks',
                  'feline_greenies_dental_treats',
                  'highland_6539_self_stick_notes',
                  'first_years_take_and_toss_straw_cup',
                  'kong_air_dog_squeakair_tennis_ball',
                  'dove_beauty_bar',
                  'one_with_nature_soap_dead_sea_mud',
                  'champion_copper_plus_spark_plug',
                  'dr_browns_bottle_brush',
                  'expo_dry_erase_board_eraser',
                  'safety_works_safety_glasses',
                  'mommys_helper_outlet_plugs',
                  'mead_index_cards',
                  'munchkin_white_hot_duck_bath_toy'};
      
      %% Features
      SURFlist_g = {balls_SURF_g,
                       book_SURF_g,
                       crayon_SURF_g,
                       duck_SURF_g,
                       frog_SURF_g,
                       glue_SURF_g,
                       hukbook_SURF_g,
                       pencil_SURF_g,
                       sharpie_SURF_g,
                       stir_stick_SURF_g,
                       catfood_SURF_g,
                       self_stick_SURF_g,
                       cups_SURF_g,
                       tennis_SURF_g,
                       dove_SURF_g,
                       soap_SURF_g,
                       plug_SURF_g,
                       brush_SURF_g,
                       eraser_SURF_g,
                       glasses_SURF_g,
                       outletplugs_SURF_g,
                       index_SURF_g,
                       rubberduck_SURF_g};
      list = containers.Map(itemlist,SURFlist_g);
      save('featureList_all.mat','list');           

      %% Thresholds
      thresholdlist = {15, % balls X
                       45, % joke book X
                       40, % crayons X-->50?
                       20, % duck X
                       30, % frog X
                       25, % glue X
                       80, % huckbook X
                       30, % pencils X
                       25, % sharpies X
                       40, % Stirsticks X--from 50 to 40
                       20, %catfood X
                       27, %self stick notes X
                       35, %cups X
                       45, %tennis X
                       40, %dove X
                       25, %soap X
                       30, %plug X
                       10, %brush X
                       100, %eraser X
                       70, %glasses X
                       45, %outletplugs X
                       40,  %indexcards
		               7}; %rubberduck
  
      list_threshold = containers.Map(itemlist,thresholdlist);
      save('thresholdList_all.mat','list_threshold');

