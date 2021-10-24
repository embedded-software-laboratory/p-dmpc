function refPath = generate_ref_path(vehid)

% commonroad_data = readstruct('LabMapCommonRoad.xml');
load('commonroad_data.mat')


switch vehid
    
    case 1
        lanelets = [6,10,14,98,96,89,113,109,162,63,59,51,49,44,2];
        refPath = [];
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
        
        
    case 2
        lanelets = [11,20,23,120,151,155,131,135,143,147,78,25,29,8];
        refPath = [];
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
       
        
    case 3
        lanelets = [14,98,96,89,113,109,162,63,59,51,49,44,2,6,10];
        refPath = [];
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
        
        
    case 4 
        lanelets = [98,96,89,113,109,162,63,59,51,49,44,2,6,10,14];
        refPath = [];
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
        
        
    case 5 
        lanelets = [93,89,113,109,162,63,59,51,47,71,67,36,105,101];
        refPath = [];
        
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
            
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
        
        
    case 6
        lanelets = [90,88,127,131,135,139,55,51,47,43,1,5,9,20,22,33,75,64,62,51,47,43,1,5,9,20];
        refPath = [];
        
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
              
        
    case 7
        lanelets = [90,86,128,132,137,146,148,79,26,31,5,9,13,97,95];
        refPath = [];
        
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 

                
    case 8
        lanelets = [128,132,137,146,148,79,26,31,5,9,13,97,95,90,86];

        refPath = [];
        
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
        
                
    case 9
        lanelets = [132,137,146,148,79,26,31,5,9,13,97,95,90,86,128];

        refPath = [];

        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
        
                
    case 10
        lanelets = [135,139,55,51,49,44,2,6,11,20,23,120,151,155,131];
        refPath = [];
        
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
        
        
    case 11
        lanelets = [140,56,52,48,2,6,11,20,23,120,151,155,131,138];
        refPath = [];
        
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 

        
    case 12
        lanelets = [55,51,47,71,67,36,105,101,93,89,85,127,131,135,139];
        refPath = [];

        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
        
        
    case 13
        lanelets = [54,47,71,67,36,105,101,93,89,85,127,131,138,140,56];
        refPath = [];
        
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
       
        
        
    case 14
        lanelets = [47,71,67,36,105,101,93,89,113,109,162,63,59,51];
        refPath = [];
        
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
      
       
    case 15
        lanelets = [43,1,5,9,20,22,33,75,64,62,51,47];
        refPath = [];
        
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
        
        
    case 16
        lanelets = [42,84,63,59,51,47,71,67,82,166,151,155,131,135,146,148,159,117,106,104,93,89,113,109,124,40,25,29,5,9,20,23];
        refPath = [];
        
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end       

    case 17
        lanelets = [106,104,93,89,113,109,124,40,25,29,5,9,20,23,42,84,63,59,51,47,71,67,82,166,151,155,131,135,146,148,159,117];
        refPath = [];
        
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
        
        
    case 18
        lanelets = [124,40,25,29,5,9,20,23,42,84,63,59,51,47,71,67,82,166,151,155,131,135,146,148,159,117,106,104,93,89,113,109];
        refPath = [];
        
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
        
        
     case 19
        lanelets = [151,155,131,135,146,148,159,117,106,104,93,89,113,109,124,40,25,29,5,9,20,23,42,84,63,59,51,47,71,67,82,166];
        refPath = [];
        
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
        
        
     case 20
        lanelets = [159,117,106,104,93,89,113,109,124,40,25,29,5,9,20,23,42,84,63,59,51,47,71,67,82,166,151,155,131,135,146,148];
        refPath = [];
        
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x(1:end);refPath_y(1:end)];
            refPath = [refPath, refPath_next];
           
        end 
        
             
end 


% figure
% plot(refPath(1,:),refPath(2,:))
% 
% xlim([0,4.5])
% ylim([0,4])


refPath = refPath';



end











































