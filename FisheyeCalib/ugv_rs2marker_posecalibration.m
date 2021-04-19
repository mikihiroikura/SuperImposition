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
    
    %HSCの動画
    %動画からFrameを保存する
    vidObj_hsc = VideoReader(video_dir_ugv_hsc);
    allFrame_hsc = read(vidObj_hsc);
    hsc_img = allFrame_hsc(:,:,:,1:img_step:end);
    
    %チェッカーボードを検出する
    [imagePoints_hsc,boardSize,imagesUsed_hsc] = detectCheckerboardPoints(hsc_img);
    worldPoints_hsc = generateCheckerboardPoints(boardSize, squareSize);
    
    %CSVからMarkerの位置姿勢を読み取る
    load setup.mat csv_dir_ugv_marker
    M =csvread(csv_dir_ugv_marker);
    M_marker = M(1:img_step:end,:);
    TransVec_marker2hsc = M_marker(:,10:12) * 1000;%単位はmm
    RotMatrix_marker2hsc = zeros(3,3,size(M_marker,1));
    for i = 1:size(M_marker,1)
        RotMatrix_marker2hsc(:,:,i) = reshape(M_marker(i,1:9),[3 3]).';
    end
    
    %RSとHSC両方で検出したフレームの洗い出し
    imageUsed_rs_hsc_marker = logical(imagesUsed_hsc .* imagesUsed_rs);
    for i = 1:size(imagePoints_hsc,3)
        if size(find(isnan(imagePoints_hsc(:,:,i))),1) > 0
           imageUsed_rs_hsc_marker(i) = 0; 
        end
    end
    for i = 1:size(imagePoints_rs,3)
        if size(find(isnan(imagePoints_rs(:,:,i))),1) > 0
           imageUsed_rs_hsc_marker(i) = 0; 
        end
    end
    imageUsed_rs_hsc_marker(TransVec_marker2hsc(:,1)==0) = 0;
    
    %再度チェッカーボード検出
    [imagePoints_rs,boardSize,imagesUsed_rs] = detectCheckerboardPoints(rs_img(:,:,:,imageUsed_rs_hsc_marker));
    worldPoints_rs = generateCheckerboardPoints(boardSize, squareSize);
    [imagePoints_hsc,boardSize,imagesUsed_hsc] = detectCheckerboardPoints(hsc_img(:,:,:,imageUsed_rs_hsc_marker));
    worldPoints_hsc = generateCheckerboardPoints(boardSize, squareSize);
    
    %再度CSVからMarkerの位置姿勢を読み取る
    TransVec_marker2hsc = M_marker(imageUsed_rs_hsc_marker,10:12) * 1000;%単位はmm
    RotMatrix_marker2hsc = zeros(3,3,size(M_marker(imageUsed_rs_hsc_marker,:),1));
    M_used = M_marker(imageUsed_rs_hsc_marker,:);
    for i = 1:size(M_used,1)
        RotMatrix_marker2hsc(:,:,i) = reshape(M_used(i,1:9),[3 3]).';
    end
    
    %チェッカーボード(World)toRSの外部パラメータの計算
    RotMatrix_rs = zeros(3,3,size(imagePoints_rs,3));
    TransVec_rs = zeros(size(imagePoints_rs,3), 3);
    for i=1:size(imagePoints_rs,3)
        [R,t] = extrinsics(imagePoints_rs(:,:,i),worldPoints_rs,rs0params);
        RotMatrix_rs(:,:,i) = R;
        TransVec_rs(i,:) = t;
    end
 
    %チェッカーボード(World)toHSCの外部パラメータの計算
    RotMatrix_hsc = zeros(3,3,size(imagePoints_hsc,3));
    TransVec_hsc = zeros(size(imagePoints_hsc,3), 3);
    for i=1:size(imagePoints_hsc,3)
        [R,t] = extrinsics(imagePoints_hsc(:,:,i),worldPoints_hsc,fisheyeParams.Intrinsics);
        RotMatrix_hsc(:,:,i) = R;
        TransVec_hsc(i,:) = t;
    end
    
    %RS-HSCの位置姿勢を計算
    RotMatrix_rs2hsc = pagemtimes(pagetranspose(RotMatrix_rs),RotMatrix_hsc);
    TransVec_rs2hsc = zeros(size(TransVec_hsc));
    for i = 1:size(TransVec_rs2hsc,1)
        TransVec_rs2hsc(i,:) = -TransVec_rs(i,:) * RotMatrix_rs2hsc(:,:,i) + TransVec_hsc(i,:);
    end
    
    
    %RS-Marker間位置姿勢の計算
    RotMatrix_rs2marker = pagemtimes(RotMatrix_rs2hsc, pagetranspose(RotMatrix_marker2hsc));
    TransVec_rs2marker = zeros(size(TransVec_hsc));
    RotMatrix_hsc2marker = pagetranspose(RotMatrix_marker2hsc);
    for i = 1:size(TransVec_rs2marker,1)
        TransVec_rs2marker(i,:) =(-TransVec_marker2hsc(i,:) + TransVec_rs2hsc(i,:)) * RotMatrix_hsc2marker(:,:,i);
    end
    
    %RS-Marker間位置姿勢の平均
    TransVec_rs2marker_mean = mean(TransVec_rs2marker,1);
    RotVec_rs2marker = zeros(size(RotMatrix_rs2marker,3), 3);
    for i = 1:size(RotMatrix_rs2marker,3)
        RotVec_rs2marker(i,:) = rotationMatrixToVector(RotMatrix_rs2marker(:,:,i));
    end
    RotVec_rs2marker_mean = mean(RotVec_rs2marker,1);
    RotMat_rs2marker_mean = rotationVectorToMatrix(RotVec_rs2marker_mean);
    
    
    %結果の保存
    save poseparams.mat TransVec_rs2hsc_mean RotMat_rs2hsc_mean TransVec_rs2marker_mean RotMat_rs2marker_mean


end