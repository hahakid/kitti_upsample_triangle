function [filtered] = distanceFilter(point,st)
    %按距离进行过滤的过滤器，内部调整xyz轴的投影范围
    %kitti 的坐标系下，车头朝向是X正，Y轴 正为左 负为右
    %boundary
%     st.bx_min=-10;%left-right
%     st.bx_max=60;
%     st.by_min=-40;%front-back
%     st.by_max=40;
%     st.bz_min=-2.5;% hight-below
%     st.bz_max=1;
%     
%     %inner un-used, filtered the ego-vehicle
%     st.ix_min=-0.8;%left-right
%     st.ix_max=0.8;
%     st.iy_min=0;%front-back
%     st.iy_max=-4.2;
    
    
    idx1=point(:,1)==0;
    idx2=point(:,2)==0;
    idx=idx1&idx2;
    point(idx,:) = [];
    
    clear idx1 idx2 
    
    idx = point(:,1)<st.x_min;
    point(idx,:) = [];
    
    idx = point(:,1)>st.x_max;
    point(idx,:) = [];
    
    idx = point(:,2)<st.y_min;
    point(idx,:) = [];
    idx = point(:,2)>st.y_max;
    point(idx,:) = [];
    
    idx = point(:,3)<st.z_min;
    point(idx,:) = [];
    idx = point(:,3)>st.z_max;
    point(idx,:) = [];
   
    
    filtered=point;
end

