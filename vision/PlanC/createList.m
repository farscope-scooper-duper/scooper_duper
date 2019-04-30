%% CreateList
% creates the list of object features linked to name and threshold, to be
% ran once if updating item list or thresholds.

%load refIMs.mat

%open featureList.mat

%itemList = cell(1,6,length('refIms.mat'));

itemID = {'crayola_64_ct',
          'mommys_helper_outlet_plugs',
          'kong_duck_dog_toy',
          'rolodex_jumbo_pencil_cup',
          'stanley_66_052',
          'genuine_joe_plastic_stir_sticks',
          'dr_browns_bottle_brush',
          'paper_mate_12_count_mirado_black_warrior',
          'expo_dry_erase_board_eraser',
          'kong_sitting_frog_dog_toy',
          'highland_6539_self_stick_notes',
          'mead_index_cards',
          'kyjen_squeakin_eggs_plush_puppies',
          'munchkin_white_hot_duck_bath_toy',
          'safety_works_safety_glasses',
          'first_years_take_and_toss_straw_cup',
          'kong_air_dog_squeakair_tennis_ball',
          'sharpie_accent_tank_style_highlighters',
          'elmers_washable_no_run_school_glue',
          'champion_copper_plus_spark_plug',
          'laugh_out_loud_joke_book',
          'feline_greenies_dental_treats'};
      
      
     %% balls object
  
      [points1, features1] = SURFextractfeatures_gray(balls_back);
      [points2, features2] = SURFextractfeatures_gray(balls_front);
      balls_points = [points1;points2];
      balls_features = [features1;features2];
      
      balls_SURF_g = {balls_points, balls_features};
      
      %% book object
      
      [points1, features1] = SURFextractfeatures_gray(book_back);
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
      
      [points1, features1] = SURFextractfeatures_gray(duck_back);
      [points2, features2] = SURFextractfeatures_gray(duck_front);
      all_points = [points1;points2];
      all_features = [features1;features2];
      
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
      
      [points1, features1] = SURFextractfeatures_gray(pencil_back);
      [points2, features2] = SURFextractfeatures_gray(pencil_front);
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
      catfood_back = imread('catfood_back.jpg');
      catfood_front = imread('catfood_front.jpg');    
      [points1, features1] = SURFextractfeatures_gray(catfood_back);
      [points2, features2] = SURFextractfeatures_gray(catfood_front);
      all_points = [points1;points2;points3;points4];
      all_features = [features1;features2;features3;features4];
      
      catfood_SURF_g = {all_points, all_features};
      
      %% index notes
      
      index_back = imread('index_back.jpg');
      index_bottom = imread('index_bottom.jpg');
      index_front = imread('index_front.jpg');
      index_top = imread('index_top.jpg');
      [points1, features1] = SURFextractfeatures_gray(index_back);
      [points2, features2] = SURFextractfeatures_gray(index_bottom);
      [points3, features3] = SURFextractfeatures_gray(index_front);
      [points4, features4] = SURFextractfeatures_gray(index_top);
      all_points = [points1;points2;points3;points4];
      all_features = [features1;features2;features3;features4];
      
      index_SURF_g = {all_points, all_features};
      
      
      %%
      
      
      
      %%
      
      SURFlist_g = {balls_SURF_g,
                       book_SURF_g,
                       crayon_SURF_g,
                       duck_SURF_g,
                       frog_SURF_g,
                       glue_SURF_g,
                       hukbook_SURF_g,
                       pencil_SURF_g,
                       sharpie_SURF_g,
                       stir_stick_SURF_g};
                   
      itemlist = {'kong_air_dog_squeakair_tennis_ball',
                  'laugh_out_loud_joke_book',
                  'crayola_64_ct',          
                  'kong_duck_dog_toy',
                  'kong_sitting_frog_dog_toy',
                  'elmers_washable_no_run_school_glue',
                  'adventures_of_huckleberry_finn_book',
                  'paper_mate_12_count_mirado_black_warrior',
                  'sharpie_accent_tank_style_highlighters',
                  'genuine_joe_plastic_stir_sticks'};
                  
      thresholdlist = {20, % balls
                       40, % joke book
                       100, % crayons
                       15, % duck
                       30, % frog
                       15, % glue
                       120, % huckbook
                       30, % pencils
                       15, % sharpies
                       60}; % Stirsticks
          
      list = containers.Map(itemlist,SURFlist_g);
      list_threshold = containers.Map(itemlist,thresholdlist);
      
      save('featureList.mat','list');
      save('thresholdList.mat','list_threshold');

%% Threshold items list

% balls ~ 20-25
% jokebook ~ 40 (v good - <15 FP and ~120 TP)
% crayon ~ 100 (v good <40 FP, 140-200 TP) 
% Duck ~ 15
% Frog ~ 30
% Glue ~ 15 (lowish response ~20 (upto 34 on back) but low FP ~5)
% hukbook ~ 120 (v strong response, 70+ on FP, nearly 400 on TP)
% pencils ~ 30 (poor, gets 20-25 false positives and true positives)
% sharpie ~ 15 (very tight on false positives- scores 20-40, but 13 on background)
% Stir sticks ~ 60 (v good, <15 fp,  120 tp)

      
