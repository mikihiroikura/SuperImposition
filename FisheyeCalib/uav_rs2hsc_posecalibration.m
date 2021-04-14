function uav_rs2hsc_posecalibration()
    load fishparams.mat fisheyeParams

    %RSの動画
    %動画からFrameを保存する
    load setup.mat video_dir_uav_rs video_dir_uav_hsc img_step squareSize
    vidObj_rs = VideoReader(video_dir_uav_rs);
    allFrame_rs = read(vidObj_rs);
    rs_img = allFrame_rs(:,:,:,1:img_step:end);
    
    %チェッカーボードを検出する
    [imagePoints_rs,boardSize,imagesUsed_rs] = detectCheckerboardPoints(rs_img);
    worldPoints_rs = generateCheckerboardPoints(boardSize, squareSize);
    
    %チェッカーボード(World)toRSの外部パラメータの計算
    RotMatrix_rs = zeros(3,3,size(imagePoints_rs,3));
    TransVec_rs = zeros(size(imagePoints_rs,3), 3);
    for i=1:size(imagePoints_rs,3)
        [R,t] = extrinsics(imagePoints_rs(:,:,i),worldPoints_rs,fisheyeParams.Intrinsics);
        RotMatrix_rs(:,:,i) = R;
        TransVec_rs(i,:) = t;
    end
    
    %HSCの動画
    %動画からFrameを保存する
    vidObj_hsc = VideoReader(video_dir_uav_hsc);
    allFrame_hsc = read(vidObj_hsc);
    hsc_img = allFrame_hsc(:,:,:,1:img_step:end);
    
    %チェッカーボードを検出する
    [imagePoints_hsc,boardSize,imagesUsed_hsc] = detectCheckerboardPoints(hsc_img);
    worldPoints_hsc = generateCheckerboardPoints(boardSize, squareSize);
    
    %チェッカーボード(World)toHSCの外部パラメータの計算
    RotMatrix_hsc = zeros(3,3,size(imagePoints_hsc,3));
    TransVec_hsc = zeros(size(imagePoints_hsc,3), 3);
    for i=1:size(imagePoints_hsc,3)
        [R,t] = extrinsics(imagePoints_hsc(:,:,i),worldPoints_hsc,fisheyeParams.Intrinsics);
        RotMatrix_hsc(:,:,i) = R;
        TransVec_hsc(i,:) = t;
    end
    
    %RS-HSC間位置姿勢の計算
    RotMatrix_rs2hsc = pagemtimes(pagetranspose(RotMatrix_rs),RotMatrix_hsc);
    TransVec_rs2hsc = -TransVec_rs + TransVec_hsc;
    
    %RS-HSC間位置姿勢の平均
    TransVec_rs2hsc_mean = mean(TransVec_rs2hsc,1);
    RotVec_rs2hsc = zeros(size(imagePoints_hsc,3), 3);
    for i = 1:size(RotMatrix_rs2hsc,3)
        RotVec_rs2hsc(:,i) = rotationMatrixToVector(RotMatrix_rs2hsc(:,:,i));
    end
    RotVec_rs2hsc_mean = mean(RotVec_rs2hsc,1);
    RotMat_rs2hsc_mean = rotationVectorToMatrix(RotVec_rs2hsc_mean);
    
    %結果の保存
    save poseparams.mat TransVec_rs2hsc_mean RotMat_rs2hsc_mean
    
    
end