function [velo] = velo_preprocess2(pixel,st,imsize)
    %按视野对额外点云进行过滤
    %pixel(pixel(:, 1) < st.x_min, :) = [];
    %clb=st.P2*st.R0_rect*st.Tr_velo_to_cam; 
    %pixel = (clb*velo')';
    
    %pixel(:, 1)   = pixel(:, 1)./pixel(:, 3); pixel(:, 2) = pixel(:, 2)./pixel(:, 3); % point's x & y are cor. to image's c & nr - r (nr: nnumber of raws)
    pixel(:, 1:2) = round([pixel(:, 1) pixel(:, 2)]);                                 % correction [r c]
    ins = (pixel(:, 1) >= 1) & (pixel(:, 1) <= imsize(1)) & ...           % index of pixels inside the image
                (pixel(:, 2) >= 1) & (pixel(:, 2) <= imsize(2));
    velo = pixel(ins, :);
end

