function [velo,rmse] = lidar_merge_icp(velo_c,velo_l)
    %����һ֡��l�������ݵ��ӵ���ǰ֡��c��
    % ���ںܶ���Ƽ�������Ὣ(visionVoxelGridFilter) ����ǿ��ɾ������αװ��Color�������(color
    % ��תuint8)�������ȡ�ڽ��д���
    
    color=[velo_c(:,4),zeros(size(velo_c(:,4))),zeros(size(velo_c(:,4)))];
    pc1=pointCloud(velo_c(:,1:3),'Color',color);
    color=[velo_l(:,4),zeros(size(velo_l(:,4))),zeros(size(velo_l(:,4)))];
    pc2=pointCloud(velo_l(:,1:3),'Color',color);
    ds=1;% �Ƿ���н�������ϡ�����
    rmse=999;% big enough
    %if ds==1
    gridSize = 0.25; %0.1
    %    fixed = pcdownsample(pc1, 'gridAverage', gridSize);%gridAverage rmse��С
    %    moving = pcdownsample(pc2, 'gridAverage', gridSize);
    %else
    %     fixed=pc1;
    %     moving=pc2;
    %end
    %[tform,movingReg,rmse] = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
    test_count=0;
    while (rmse>3 && test_count<5) % ����С��2 ֹͣ
        test_count=test_count+1;
        fixed = pcdownsample(pc1, 'gridAverage', gridSize);
        moving = pcdownsample(pc2, 'gridAverage', gridSize);
        %fixed = pcdownsample(pc1, 'nonuniformGridSample', 200);
        %moving = pcdownsample(pc2, 'nonuniformGridSample', 200);
        
        %fixed=pc1;moving=pc2;
        [tform,movingReg,rmse] = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
        %pc2 = pctransform_modified(pc2,tform);%��ת
        pc2 = pctransform(pc2,tform);%��ת
        
        %rmse,test_count
    end
    
    mergeSize = 0.01;
    %tic;
    %point=fix(pc2.Location);
    %pc2=pointCloud(point,'Color',pc2.Color);
    
    velo = pcmerge(pc1, pc2, mergeSize);%�ϲ�
    Ins=double(velo.Color(:,1))/255;% ��ȡǿ��ֵ����ԭ
    velo=pointCloud(velo.Location,'Intensity',Ins);
    
    %toc;
    %subplot(3,1,1);
    %pcshow(pc1);
    %subplot(3,1,2);
    %pcshow(pc2);
    %subplot(3,1,3);
    %pcshow(velo);
end

