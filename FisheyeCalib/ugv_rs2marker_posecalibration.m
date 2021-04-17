function ugv_rs2marker_posecalibration()
    load fishparams.mat fisheyeParams
    load poseparams.mat TransVec_rs2hsc_mean RotMat_rs2hsc_mean
    load rs0params.mat rs0params
    
    %RSの動画
    %動画からFrameを保存する
    load setup.mat video_dir_ugv_rs video_dir_ugv_hsc img_step squareSize
    vidObj_rs = VideoReader(video_dir_ugv_rs);
    allFrame_rs = read(vidObj_rs);
    rs_img = allFrame_rs(:,:,:,1:img_step:end);
    
    %チェッカーボードを検出する
    [imagePoints_rs,boardSize,imagesUsed_rs] = detectCheckerboardPoints(rs_img);
    worldPoints_rs = generateCheckerboardPoints(boardSize, squareSize);
    
    %チェッカーボード(World)toRSの外部パラメータの計算
    RotMatrix_rs = zeros(3,3,size(imagePoints_rs,3));
    TransVec_rs = zeros(size(imagePoints_rs,3), 3);
    for i=1:size(imagePoints_rs,3)
        [R,t] = extrinsics(imagePoints_rs(:,:,i),worldPoints_rs,rs0params);
        RotMatrix_rs(:,:,i) = R;
        TransVec_rs(i,:) = t;
    end
    
    %HSCの動画
    %動画からFrameを保存する
    vidObj_hsc = VideoReader(video_dir_ugv_hsc);
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
    
    %CSVからMarkerの位置姿勢を読み取る
    load setup.mat csv_dir_ugv_marker
    M =csvread(csv_dir_ugv_marker);
    TransVec_hsc2marker = M(:,10:12);
    RotMatrix_hsc2marker = zeros(3,3,size(M,1));
    for i = 1:size(M,1)
        RotMatrix_hsc2marker(:,:,i) = reshape(M(i,:),[3 3]).';
    end
    
    
    %RS-HSCの位置姿勢を計算
    RotMatrix_rs2hsc = pagemtimes(pagetranspose(RotMatrix_rs),RotMatrix_hsc);
    TransVec_rs2hsc = -TransVec_rs + TransVec_hsc;
    
    
    %RS-Marker間位置姿勢の計算
    TransVec_hsc2marker_nonzero = TransVec_hsc2marker(any(TransVec_hsc2marker,2),:);
    TransVec_rs2hsc_nonzero = TransVec_rs2hsc(any(TransVec_hsc2marker,2),:);
    RotMatrix_hsc2marker_nonzero = RotMatrix_hsc2marker(:,:,any(TransVec_hsc2marker,2));
    RotMatrix_rs2hsc_nonzero = RotMatrix_rs2hsc(:,:,any(TransVec_hsc2marker,2));
    TransVec_rs2marker = TransVec_hsc2marker_nonzero + TransVec_rs2hsc_nonzero;
    RotMatrix_rs2marker = pagemtimes(RotMatrix_rs2hsc_nonzero, RotMatrix_hsc2marker_nonzero);
    
    %RS-Marker間位置姿勢の平均
    TransVec_rs2marker_mean = mean(TransVec_rs2marker,1);
    RotVec_rs2marker = zeros(size(RotMatrix_rs2marker,3), 3);
    for i = 1:size(RotMatrix_rs2marker,3)
        RotVec_rs2marker(:,i) = rotationMatrixToVector(RotMatrix_rs2marker(:,:,i));
    end
    RotVec_rs2marker_mean = mean(RotVec_rs2marker,1);
    RotMat_rs2marker_mean = rotationVectorToMatrix(RotVec_rs2marker_mean);
    
    
    %結果の保存
    save poseparams.mat TransVec_rs2hsc_mean RotMat_rs2hsc_mean TransVec_rs2marker_mean RotMat_rs2marker_mean


end