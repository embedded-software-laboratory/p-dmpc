function refPath = generateRefPath(vehid)

commonroad_data = readstruct('LabMapCommonRoad.xml');

% lanelet_idx = 2;
% refPath = [];
% for nlanelets = 1:30
%     refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelet_idx).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelet_idx).rightBound.point.x));
%     refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelet_idx).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelet_idx).rightBound.point.y));
%     refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
% %     refPath_next = [refPath_x;refPath_y];
%     refPath = [refPath, refPath_next];
%     successor_idx = horzcat(commonroad_data.lanelet(lanelet_idx).successor.refAttribute);
% %     disp(['successor_idx is:',num2str(successor_idx)])
%     %     lanelet_idx = successor_idx(1);
%     if length(successor_idx)>1
% %         lanelet_idx = randsample(successor_idx,1);
%         lanelet_idx = successor_idx(2);
%     else
%         lanelet_idx = successor_idx;
%     end
% %     disp(['lanelet_idx is:',num2str(lanelet_idx)])
% end

switch vehid
    
    case 1
%         lanelets = [1,5,9,13,97,93,89,85,127,131,135,139,55,51,80,72,70,83,65,62,51];
        lanelets = [2,6,11,13,97,93,89,85,127,131,135,139,55,51,47,71,67,83,63,59,51,47,71];
        refPath = [];
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
            
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end
        
    case 2
%         lanelets = [7,9,20,23,42,84,65,62,51,47,73,70,82,166,154,157,131,135,146,149,167,154];
        lanelets = [10,14,98,96,89,113,109,162,63,59,51,49,44,2,6,10];
        refPath = [];
        for nlanelets = 1:length(lanelets)
            
            if nlanelets == 162 ||163||121||120||78||79||36||37
                refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
                refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));

                refPath_next = [refPath_x([1,4,7,10]);refPath_y([1,4,7,10])];
                refPath = [refPath, refPath_next];
            else
                refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
                refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));

                refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
                refPath = [refPath, refPath_next];
            end
           
        end
        
    case 3
        lanelets = [9,20,23,42,84,65,62,51,47,73,70,82,166,154,157,131,135,146,148,159,117,106,104,93,91,115,110,118,106];
        refPath = [];
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
            
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end
        
    case 4
   
%         lanelets = [98,96,89,85,127,131,135,139,55,51,47,43,1,5,9,20,22,33,75,64,62,51];
        lanelets = [98,94,90,86,128,132,136,140,56,52,48,44,2,6,11,20,23,42,84,63,59,51,49,44,2,6,11,20,23];

        refPath = [];
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end
        
    case 5 %figure of 8
     
%         lanelets = [93,89,115,112,162,65,60,81,80,72,70,36,105,101,93];
%         lanelets = [93,89,113,109,162,63,59,51,47,71,67,36,105,101,93,89,113,109,162];
        lanelets = [96,89,113,109,162,63,59,51,49,44,2,6,10,14,98,96,89,113,109,162];
        refPath = [];
        
        for nlanelets = 1:length(lanelets)
            
            if nlanelets == 162 || 163 || 121 || 120 || 78 || 79 || 36 || 37
                refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
                refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));

                refPath_next = [refPath_x([1,4,7,10]);refPath_y([1,4,7,10])];
                refPath = [refPath, refPath_next];
            else
                refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
                refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));

%                 refPath_next = [refPath_x([1,4,7,10]);refPath_y([1,4,7,10])];
                refPath_next = [refPath_x(:);refPath_y(:)];
                refPath = [refPath, refPath_next];
            end
           
        end
        
        
    case 6
     
        lanelets = [89,85,127,131,135,139,55,51,47,43,1,5,9,20,22,33,75,64,62,51];
        refPath = [];
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end 
                
    case 7
     
        lanelets = [85,127,131,165,144,149,78,28,30,38,39,18,23,120,154,156,164,165,144];
        refPath = [];
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end
        
                
    case 8

        lanelets = [127,131,165,144,149,78,28,30,38,39,18,23,120,154,156,164,165,144];
        refPath = [];
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end
        
                
    case 9

        lanelets = [128,132,137,146,149,78,28,30,38,39,18,23,120,151,155,131,165,144,149,78];
        refPath = [];
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end
        
                
    case 10

        lanelets = [141,55,51,47,43,1,5,9,20,23,120,155,131,135,139,55];
        refPath = [];
        for nlanelets = 1:length(lanelets)
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end
            
        
    case 11
     
        lanelets = [141,55,51,80,72,70,36,105,101,93,89,85,127,131,135,146,148,159,117,106,104];
        N = length(lanelets);
        refPath = [];
        for nlanelets = 1:N
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end

        
    case 12
     
        lanelets = [55,51,80,72,70,36,105,101,93,89,85,127,131,135,139,55,51,80,72,70,36,105,101];
        N = length(lanelets);
        refPath = [];
        for nlanelets = 1:N
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end

        
        
    case 13
     
        lanelets = [51,80,72,70,36,105,101,93,89,115,110,118,106,104,93];
        N = length(lanelets);
        refPath = [];
        for nlanelets = 1:N
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end
       
        
        
    case 14
     
        lanelets = [50,73,68,77,161,152,156,164,165,144,148,117,106,104,93,89,85,127,131,165,144,148,117,106];
        N = length(lanelets);
        refPath = [];
        for nlanelets = 1:N
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end
      
       
    case 15
     
        lanelets = [43,1,5,9,13,97,93,89,85,127,131,135,146,149,78,25,29,5,9,13,97,93,89];
        N = length(lanelets);
        refPath = [];
        for nlanelets = 1:N
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end
%         refPath_x_end = 1/2*(horzcat(commonroad_data.lanelet(lanelets(N)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(N)).rightBound.point.x));
%         refPath_y_end = 1/2*(horzcat(commonroad_data.lanelet(lanelets(N)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(N)).rightBound.point.y));
% 
%         refPath_next = [refPath_x_end([1,6,10]);refPath_y_end([1,6,10])];
% %             refPath_next = [refPath_x;refPath_y];
%         refPath = [refPath, refPath_next];          
%         
        
    case 16
     
        lanelets = [33,75,64,60,81,80,72,68,77,161,152,156,164,165,144,148,159,117,106,102,123,122,114,110,119,35,26,30,38,39,18,22,33,75,64];
        N = length(lanelets);
        refPath = [];
        for nlanelets = 1:N
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end        

    case 17
     
        lanelets = [117,106,102,123,122,114,110,119,35,26,30,38,39,18,22,33,75,64,60,81,80,72,68,77,161,152,156,164,165,144,148,159,117,106,102,123];
        N = length(lanelets);
        refPath = [];
        for nlanelets = 1:N
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end 
        
        
    case 18
     
        lanelets = [118,106,102,123,122,114,110,119,35,26,30,38,39,18,22,33,75,64,60,81,80,72,68,77,161,152,156,164,165,144,148,159,117,106,102,123];
        N = length(lanelets);
        refPath = [];
        for nlanelets = 1:N
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end 
        
        
     case 19
     
        lanelets = [152,156,164,165,144,148,159,117,106,102,123,122,114,110,119,35,26,30,38,39,18,22,33,75,64,60,81,80,72,68,77,161,152,156,164,165];
        N = length(lanelets);
        refPath = [];
        for nlanelets = 1:N
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end 
        
        
     case 20
     
%         lanelets = [159,117,106,102,123,122,114,110,119,35,26,30,38,39,18,22,33,75,64,60,81,80,72,68,77,161,152,156,164,165,144,148,159,117,106];
        lanelets = [159,117,106,104,93,89,113,109,124,40,25,29,5,9,20,23,42,84,63,59,51,47,71,67,82,166,151,155,131,135,146,148,159,117,106];
%         lanelets = [159,117,106,104,93,89,113,109,124,40,25,29,5,39,18,22,33,75,60,81,47,73,68,77,161,152,157,131,165,144,148,159,117,106];
        N = length(lanelets);
        refPath = [];
        for nlanelets = 1:N
            
            refPath_x = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.x) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.x));
            refPath_y = 1/2*(horzcat(commonroad_data.lanelet(lanelets(nlanelets)).leftBound.point.y) + horzcat(commonroad_data.lanelet(lanelets(nlanelets)).rightBound.point.y));
      
            refPath_next = [refPath_x([1,5,9]);refPath_y([1,5,9])];
            refPath = [refPath, refPath_next];
           
        end 
        
             
end 


figure
plot(refPath(1,:),refPath(2,:))

xlim([0,4.5])
ylim([0,4])


refPath = refPath';



end











































