function [velo,rmse_direct,rmse] = lidar_merge_direct(velo_c,velo_l)
    %直接错位叠加
    velo_l(:,1)=velo_l(:,1)-2;% 第一列为x，-意味着整体向上平移n个像素，而当前图像是上部空白，所以不会引起第二行
    px=velo_l(:,2)<=0;
    velo_l(px,:)=[];
    color=[velo_c(:,4),zeros(size(velo_c(:,4))),zeros(size(velo_c(:,4)))];
    pc1=pointCloud(velo_c(:,1:3),'Color',color);
    color=[velo_l(:,4),zeros(size(velo_l(:,4))),zeros(size(velo_l(:,4)))];
    pc2=pointCloud(velo_l(:,1:3),'Color',color);
    gridStep=0.25;
    velo=pcmerge(pc1,pc2,gridStep);
    Ins=double(velo.Color(:,1))/255;% 抽取强度值，还原
    velo=pointCloud(velo.Location,'Intensity',Ins);
    %%%%%%%%%%%%%%%%%%%%%%% RMSE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %当前计算方式太粗暴，利用一次icp进行估计，但是不计算误差
    if pc1.Count<pc2.Count
        squaredError = sum((pc1.Location(:,1:3)-pc2.Location(1:pc1.Count,1:3)).^2, 2);
    else
        squaredError = sum((pc1.Location(1:pc2.Count,1:3)- pc2.Location(:,1:3)).^2, 2);
    end
    rmse_direct=sqrt(sum(squaredError)/length(squaredError));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %利用icp 只评估不旋转，误差会小很多
    color=[velo_c(:,4),zeros(size(velo_c(:,4))),zeros(size(velo_c(:,4)))];
    fixed=pointCloud(velo_c(:,1:3),'Color',color); % 固定帧，先转点云
    fixed=pcdownsample(fixed,'gridAverage',gridStep);%降采样
    color=[velo_l(:,4),zeros(size(velo_l(:,4))),zeros(size(velo_l(:,4)))];
    moving=pointCloud(velo_l(:,1:3),'Color',color);  % 运动帧，先转点云
    moving=pcdownsample(moving,'gridAverage',gridStep); %降采样
    [~,~,rmse] = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
       
    %velo1=pcdenoise(velo);
    %pcshowpair(velo,velol,'VerticalAxis','Y','VerticalAxisDir','Down');
    %subplot(2,1,1);
    %pcshow(velo);
    %view(90,90);
    %subplot(2,1,2);
    %pcshow(velo1);
    %view(90,90);
    %rmse=0;
end

