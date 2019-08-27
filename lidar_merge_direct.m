function [velo,rmse_direct,rmse] = lidar_merge_direct(velo_c,velo_l)
    %ֱ�Ӵ�λ����
    velo_l(:,1)=velo_l(:,1)-2;% ��һ��Ϊx��-��ζ����������ƽ��n�����أ�����ǰͼ�����ϲ��հף����Բ�������ڶ���
    px=velo_l(:,2)<=0;
    velo_l(px,:)=[];
    color=[velo_c(:,4),zeros(size(velo_c(:,4))),zeros(size(velo_c(:,4)))];
    pc1=pointCloud(velo_c(:,1:3),'Color',color);
    color=[velo_l(:,4),zeros(size(velo_l(:,4))),zeros(size(velo_l(:,4)))];
    pc2=pointCloud(velo_l(:,1:3),'Color',color);
    gridStep=0.25;
    velo=pcmerge(pc1,pc2,gridStep);
    Ins=double(velo.Color(:,1))/255;% ��ȡǿ��ֵ����ԭ
    velo=pointCloud(velo.Location,'Intensity',Ins);
    %%%%%%%%%%%%%%%%%%%%%%% RMSE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %��ǰ���㷽ʽ̫�ֱ�������һ��icp���й��ƣ����ǲ��������
    if pc1.Count<pc2.Count
        squaredError = sum((pc1.Location(:,1:3)-pc2.Location(1:pc1.Count,1:3)).^2, 2);
    else
        squaredError = sum((pc1.Location(1:pc2.Count,1:3)- pc2.Location(:,1:3)).^2, 2);
    end
    rmse_direct=sqrt(sum(squaredError)/length(squaredError));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %����icp ֻ��������ת������С�ܶ�
    color=[velo_c(:,4),zeros(size(velo_c(:,4))),zeros(size(velo_c(:,4)))];
    fixed=pointCloud(velo_c(:,1:3),'Color',color); % �̶�֡����ת����
    fixed=pcdownsample(fixed,'gridAverage',gridStep);%������
    color=[velo_l(:,4),zeros(size(velo_l(:,4))),zeros(size(velo_l(:,4)))];
    moving=pointCloud(velo_l(:,1:3),'Color',color);  % �˶�֡����ת����
    moving=pcdownsample(moving,'gridAverage',gridStep); %������
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

