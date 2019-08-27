function [upsampled] = Fun_upsample(pixel,im,st,interp_method)
    
%     velo(velo(:, 1) < st.x_min, :) = [];
%     clb=st.P2*st.R0_rect*st.Tr_velo_to_cam; 
%     pixel = (clb * velo')';
%     
%     pixel(:, 1)   = pixel(:, 1)./pixel(:, 3); pixel(:, 2) = pixel(:, 2)./pixel(:, 3); % point's x & y are cor. to image's c & nr - r (nr: nnumber of raws)
%     pixel(:, 1:2) = round([pixel(:, 2) pixel(:, 1)]);                                 % correction [r c]
%     ins = (pixel(:, 1) >= 1) & (pixel(:, 1) <= size(im, 1)) & ...           % index of pixels inside the image
%                 (pixel(:, 2) >= 1) & (pixel(:, 2) <= size(im, 2));
%     pixel = pixel(ins, :);                                                    % [r c depth reflectance]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%interpolant based%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% post process: unique pixel with closest distances
    [v, i]     = sort(pixel(:, 3));                               % sort based on depth
    A          = [pixel(i, 1:2), v, pixel(i, 4)];                 % sorted px [r c depth reflectance]
    [C, ia, ~] = unique(A(:, 1:2), 'rows');                       % unique value (keeps closest points)
    pixel      = [C, A(ia, 3), A(ia, 4)];                         % [r c depth reflectance], C = A(ia,:) and A = C(ic,:)

%% sparse maps
    l_ind      = sub2ind([size(im, 1), size(im, 2)], pixel(:, 1), pixel(:, 2));    % [r, c] to linear index
    dms = zeros(size(im, 1), size(im, 2));                        % depth map
    dms(l_ind) = pixel(:, 3);
    rms = zeros(size(im, 1), size(im, 2));                        % reflectance map
    rms(l_ind) = pixel(:, 4);

%% build indeces for interpolation
    [r, c] = size(rms);                                           % build indeces for interpolation
    [row, col] = meshgrid(1:r, 1:c); 
    ind = sub2ind([r, c], row, col);                              % [r, c] to linear index

%% interpolation reflectance-map 
    IFR = scatteredInterpolant(pixel(:, 1), pixel(:, 2), pixel(:, 4), interp_method); % build reflectance interpolation function
    reflectance_data = IFR(row, col);                             % compute interpolated indices
    rmd = zeros(size(rms));
    rmd(ind) = reflectance_data;                                  % dense reflectancemap map

%% interpolation depth-map
    IFD = scatteredInterpolant(pixel(:, 1), pixel(:, 2), pixel(:, 3), interp_method);  % build depth interpolation function
    depth_data = IFD(row, col);                                   % compute interpolated indices
    dmd = zeros(size(dms));
    dmd(ind) = depth_data;                                        % dense depth map

%% normalize depth-map
    dmn = (st.x_max*(dmd-st.x_min))./(dmd*(st.x_max-st.x_min));   % depth-map encoding: RangeInverse
    dmn(dmn < 0) = 0; dmn(dmn > 1) = 1; 

%% mask the result to the ROI
    mask   = zeros(size(dms));                                % initialize mask
    for i  = 1:c                                              % build the mask
    vc     = dms(:, i) ~= 0;
    [~, l] = max(vc);
    mask(l:end, i) = 1;
    end
    he     = strel('rectangle', [2, 10]);                     % horizontal extension
    mask   = imdilate(mask, he);                              % by Dilation

    dm  = uint8(mask.*(255*dmn));                             % mask the depth map
    rm  = uint8(mask.*(255*rmd));                             % mask the reflectance map
   
    
    %%%%%%%%%%%bilateral filter%%%%%%%%%%%%%%%
    %pixel=pixel';
    pixel(:, 1:2) = round([pixel(:, 2) pixel(:, 1)]);% turn back for line 8
    Pts = zeros(size(pixel,1),4);
    Pts = sortrows(pixel,2);% 
    c_px = floor(min(pixel(:,2)));% boundary
    %if c_px<=0
    %    c_px=1;
    %end
    i_size = size(im(c_px:end,:,1));% 
    Ima3D = zeros( size(im(:,:,1)) );% 
    Ima3D(c_px:end,:) = fun_dense3D(Pts,[c_px i_size]); % c++
    %Ima3D(c_px:end,:) = fun_dense3D_k3(Pts,[c_px i_size]); % c++
    idxx=find(Ima3D==0); % find all 1.5
    Ima3D(idxx)=max(max(Ima3D)); % set 1 to these indexes
    bm = uint8( 255*Ima3D/max(max(Ima3D)) ); % :)% uint8
    
    %{
    %%%%%%%%%%%%%%%%%%show current frame%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    subplot(4,1,1);
    imshow(im);
    subplot(4,1,2);
    imshow(dm);
    subplot(4,1,3);
    imshow(rm);
    subplot(4,1,4);
    imshow(bm);
    
    %}
    %%%%%%%%%%%%%%%%%%%%%%%combine%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %figure(2);
    upsampled(:,:,1)=dm;
    upsampled(:,:,2)=rm;
    upsampled(:,:,3)=bm;
    %imshow(upsampled);
    
end

