function [] = compare(img,velo_before,velo_after)

figure(1);

subplot(211);
image(img);
hold on;
coord=fix(velo_before);
[len,w]=size(coord(:,3));
emp=zeros(len,1);
%collor=[coord(:,3)/max(coord(:,3)) emp emp];% ÅäÉ«
collor=[coord(:,2)/max(coord(:,2)) coord(:,1)/max(coord(:,1)) coord(:,3)/max(coord(:,3))];
%hold on;
scatter(coord(:,2),coord(:,1),1,collor);
%xlim([0,1200]);
%ylim([0 374]);

subplot(212);
image(img);
hold on;
coord=fix(velo_after);
[len,w]=size(coord(:,3));
emp=zeros(len,1);
%collor=[coord(:,3)/max(coord(:,3)) emp emp];% ÅäÉ«
collor=[coord(:,2)/max(coord(:,2)) coord(:,1)/max(coord(:,1)) coord(:,3)/max(coord(:,3))];% ÅäÉ«
scatter(coord(:,2),coord(:,1),1,collor);
%xlim([0,1242]);
%ylim([0 374]);
end

