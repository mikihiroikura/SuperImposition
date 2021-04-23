function uav_rs2hsc_posecalibration()
    load fishparams.mat fisheyeParams
    load rs1params.mat rs1params

    %RSの動画
    %動画からFrameを保存する
    load setup.mat video_dir_uav_rs video_dir_uav_hsc img_step squareSize
    vidObj_uavrs = VideoReader(video_dir_uav_rs);
    allFrame_uavrs = read(vidObj_uavrs);
    uavrs_img = allFrame_uavrs(:,:,:,1:img_step:end);
    
    %チェッカーボードを検出する
    [imagePoints_uavrs,boardSize,imagesUsed_uavrs] = detectCheckerboardPoints(uavrs_img);
    worldPoints_uavrs = generateCheckerboardPoints(boardSize, squareSize);
       
    %HSCの動画
    %動画からFrameを保存する
    vidObj_hsc = VideoReader(video_dir_uav_hsc);
    allFrame_hsc = read(vidObj_hsc);
    hsc_img = allFrame_hsc(:,:,:,1:img_step:end);
    
    %チェッカーボードを検出する
    [imagePoints_hsc,boardSize,imagesUsed_hsc] = detectCheckerboardPoints(hsc_img);
    worldPoints_hsc = generateCheckerboardPoints(boardSize, squareSize);
    
    %RSとHSC両方で検出したフレームの洗い出し
    imageUsed_uavrs_and_hsc = logical(imagesUsed_hsc .* imagesUsed_uavrs);
    cnt = 0;
    for i = 1:size(imagesUsed_hsc,1)
        if imagesUsed_hsc(i)
            cnt = cnt + 1;
            if size(find(isnan(imagePoints_hsc(:,:,cnt))),1) > 0
               imageUsed_uavrs_and_hsc(i) = 0; 
            end
        end
    end
    cnt = 0;
    for i = 1:size(imagesUsed_uavrs,1)
        if imagesUsed_uavrs(i)
            cnt = cnt + 1;
            if size(find(isnan(imagePoints_uavrs(:,:,cnt))),1) > 0
               imageUsed_uavrs_and_hsc(i) = 0; 
            end
        end
    end
    
    %再度チェッカーボード検出
    [imagePoints_uavrs,boardSize,imagesUsed_uavrs] = detectCheckerboardPoints(uavrs_img(:,:,:,imageUsed_uavrs_and_hsc));
    worldPoints_uavrs = generateCheckerboardPoints(boardSize, squareSize);
    [imagePoints_hsc,boardSize,imagesUsed_hsc] = detectCheckerboardPoints(hsc_img(:,:,:,imageUsed_uavrs_and_hsc));
    worldPoints_hsc = generateCheckerboardPoints(boardSize, squareSize);
    
    
    %チェッカーボード(World)toRSの外部パラメータの計算
    RotMatrix_uavrs = zeros(3,3,size(imagePoints_uavrs,3));
    TransVec_uavrs = zeros(size(imagePoints_uavrs,3), 3);
    for i=1:size(imagePoints_uavrs,3)
        [R,t] = extrinsics(imagePoints_uavrs(:,:,i),worldPoints_uavrs,rs1params);
        RotMatrix_uavrs(:,:,i) = R;
        TransVec_uavrs(i,:) = t;
    end
    
    %チェッカーボード(World)toHSCの外部パラメータの計算
    RotMatrix_hsc = zeros(3,3,size(imagePoints_hsc,3));
    TransVec_hsc = zeros(size(imagePoints_hsc,3), 3);
    for i=1:size(imagePoints_hsc,3)
        [R,t] = extrinsics(imagePoints_hsc(:,:,i),worldPoints_hsc,fisheyeParams.Intrinsics);
        RotMatrix_hsc(:,:,i) = R;
        TransVec_hsc(i,:) = t;
    end
    
    %RS-HSC間位置姿勢の計算
    RotMatrix_uavrs2hsc = pagemtimes(pagetranspose(RotMatrix_uavrs),RotMatrix_hsc);
    TransVec_uavrs2hsc = zeros(size(TransVec_hsc));
    for i = 1:size(TransVec_uavrs2hsc,1)
        TransVec_uavrs2hsc(i,:) = -TransVec_uavrs(i,:) * RotMatrix_uavrs2hsc(:,:,i) + TransVec_hsc(i,:);
    end
    
    %RS-HSC間位置姿勢の平均
    TransVec_uavrs2hsc_mean = mean(TransVec_uavrs2hsc,1);
    RotVec_uavrs2hsc = zeros(size(imagePoints_hsc,3), 3);
    for i = 1:size(RotMatrix_uavrs2hsc,3)
        RotVec_uavrs2hsc(i,:) = rotationMatrixToVector(RotMatrix_uavrs2hsc(:,:,i));
    end
    RotVec_uavrs2hsc_mean = mean(RotVec_uavrs2hsc,1);
    RotMat_uavrs2hsc_mean = rotationVectorToMatrix(RotVec_uavrs2hsc_mean);
    
    %結果の保存
    save poseparams.mat TransVec_uavrs2hsc_mean RotMat_uavrs2hsc_mean
    
    
end