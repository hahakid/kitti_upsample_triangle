function [velo_ng] = removeground(velo)
    figure(1);
    subplot(2,1,1);
    color=[velo(:,4),zeros(size(velo(:,4))),zeros(size(velo(:,4)))];
    pc=pointCloud(velo(:,1:3),'Color',color);
    scatter3(velo(:,1),velo(:,2),velo(:,3),1,velo(:,4));
    ground = segmentGroundFromLidarData(pc);
    subplot(2,1,2);
    scatter3(ground(:,1),ground(:,2),ground(:,3),1,ground(:,4));
    
    
end

