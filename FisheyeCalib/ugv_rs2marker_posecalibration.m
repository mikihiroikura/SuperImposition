function ugv_rs2marker_posecalibration()
    load fishparams.mat fisheyeParams
    load rs0params.mat rs0params rs0_RTcol2dpt
    
    %RSの動画
    %動画からFrameを保存する
    load setup.mat video_dir_ugv_rs video_dir_ugv_hsc img_step squareSize time_margin
    vidObj_ugvrs = VideoReader(video_dir_ugv_rs);
    allFrame_ugvrs = read(vidObj_ugvrs);
    ugvrs_img = allFrame_ugvrs(:,:,:,int16(time_margin*vidObj_ugvrs.FrameRate):img_step:int16((vidObj_ugvrs.Duration-time_margin)*vidObj_ugvrs.FrameRate));
    
    %チェッカーボードを検出する
    [imagePoints_ugvrs,boardSize,imagesUsed_ugvrs] = detectCheckerboardPoints(ugvrs_img);
    worldPoints_ugvrs = generateCheckerboardPoints(boardSize, squareSize);
    
    %HSCの動画
    %動画からFrameを保存する
    vidObj_hsc = VideoReader(video_dir_ugv_hsc);
    allFrame_hsc = read(vidObj_hsc);
    hsc_img = allFrame_hsc(:,:,:,int16(time_margin*vidObj_hsc.FrameRate):img_step:int16((vidObj_hsc.Duration-time_margin)*vidObj_hsc.FrameRate));
    
    %チェッカーボードを検出する
    [imagePoints_hsc,boardSize,imagesUsed_hsc] = detectCheckerboardPoints(hsc_img);
    worldPoints_hsc = generateCheckerboardPoints(boardSize, squareSize);
    
    %CSVからMarkerの位置姿勢を読み取る
    load setup.mat csv_dir_ugv_marker
    M =csvread(csv_dir_ugv_marker);
    M_marker = M(int16(time_margin*vidObj_hsc.FrameRate):img_step:int16((vidObj_hsc.Duration-time_margin)*vidObj_hsc.FrameRate),:);
    TransVec_marker2hsc = M_marker(:,10:12) * 1000;%単位はmm
    RotMatrix_marker2hsc = zeros(3,3,size(M_marker,1));
    for i = 1:size(M_marker,1)
        RotMatrix_marker2hsc(:,:,i) = reshape(M_marker(i,1:9),[3 3]).';
    end
    
    %RSとHSC両方で検出したフレームの洗い出し
    imageUsed_ugvrs_hsc_marker = logical(imagesUsed_hsc .* imagesUsed_ugvrs);
    cnt = 0;
    for i = 1:size(imagesUsed_hsc,1)
        if imagesUsed_hsc(i)
            cnt = cnt + 1;
            if size(find(isnan(imagePoints_hsc(:,:,cnt))),1) > 0
               imageUsed_ugvrs_hsc_marker(i) = 0; 
            end
        end
    end
    cnt = 0;
    for i = 1:size(imagesUsed_ugvrs,1)
        if imagesUsed_ugvrs(i)
            cnt = cnt + 1;
            if size(find(isnan(imagePoints_ugvrs(:,:,cnt))),1) > 0
               imageUsed_ugvrs_hsc_marker(i) = 0; 
            end
        end
    end
    imageUsed_ugvrs_hsc_marker(TransVec_marker2hsc(:,1)==0) = 0;
    
    %再度チェッカーボード検出
    [imagePoints_ugvrs,boardSize,imagesUsed_ugvrs] = detectCheckerboardPoints(ugvrs_img(:,:,:,imageUsed_ugvrs_hsc_marker));
    worldPoints_ugvrs = generateCheckerboardPoints(boardSize, squareSize);
    [imagePoints_hsc,boardSize,imagesUsed_hsc] = detectCheckerboardPoints(hsc_img(:,:,:,imageUsed_ugvrs_hsc_marker));
    worldPoints_hsc = generateCheckerboardPoints(boardSize, squareSize);
    
    %再度CSVからMarkerの位置姿勢を読み取る
    TransVec_marker2hsc = M_marker(imageUsed_ugvrs_hsc_marker,10:12) * 1000;%単位はmm
    RotMatrix_marker2hsc = zeros(3,3,size(M_marker(imageUsed_ugvrs_hsc_marker,:),1));
    M_used = M_marker(imageUsed_ugvrs_hsc_marker,:);
    for i = 1:size(M_used,1)
        RotMatrix_marker2hsc(:,:,i) = reshape(M_used(i,1:9),[3 3]).';
    end
    
    %チェッカーボード(World)toRSCOLの外部パラメータの計算
    RotMatrix_ugvrs = zeros(3,3,size(imagePoints_ugvrs,3));
    TransVec_ugvrs = zeros(size(imagePoints_ugvrs,3), 3);
    for i=1:size(imagePoints_ugvrs,3)
        [R,t] = extrinsics(imagePoints_ugvrs(:,:,i),worldPoints_ugvrs,rs0params);
        RotMatrix_ugvrs(:,:,i) = R;
        TransVec_ugvrs(i,:) = t;
    end
 
    %チェッカーボード(World)toHSCの外部パラメータの計算
    RotMatrix_hsc = zeros(3,3,size(imagePoints_hsc,3));
    TransVec_hsc = zeros(size(imagePoints_hsc,3), 3);
    for i=1:size(imagePoints_hsc,3)
        [R,t] = extrinsics(imagePoints_hsc(:,:,i),worldPoints_hsc,fisheyeParams.Intrinsics);
        RotMatrix_hsc(:,:,i) = R;
        TransVec_hsc(i,:) = t;
    end
    
    %RSCOL-HSCの位置姿勢を計算
    RotMatrix_ugvrs2hsc = pagemtimes(pagetranspose(RotMatrix_ugvrs),RotMatrix_hsc);
    TransVec_ugvrs2hsc = zeros(size(TransVec_hsc));
    for i = 1:size(TransVec_ugvrs2hsc,1)
        TransVec_ugvrs2hsc(i,:) = -TransVec_ugvrs(i,:) * RotMatrix_ugvrs2hsc(:,:,i) + TransVec_hsc(i,:);
    end
    
    
    %RSCOL-Marker間位置姿勢の計算
    RotMatrix_ugvrs2marker = pagemtimes(RotMatrix_ugvrs2hsc, pagetranspose(RotMatrix_marker2hsc));
    TransVec_ugvrs2marker = zeros(size(TransVec_hsc));
    RotMatrix_hsc2marker = pagetranspose(RotMatrix_marker2hsc);
    for i = 1:size(TransVec_ugvrs2marker,1)
        TransVec_ugvrs2marker(i,:) =(-TransVec_marker2hsc(i,:) + TransVec_ugvrs2hsc(i,:)) * RotMatrix_hsc2marker(:,:,i);
    end
    
    %RSCOL-Marker間位置姿勢の平均
    TransVec_ugvrs2marker_mean = mean(TransVec_ugvrs2marker,1);
    RotVec_ugvrs2marker = zeros(size(RotMatrix_ugvrs2marker,3), 3);
    for i = 1:size(RotMatrix_ugvrs2marker,3)
        RotVec_ugvrs2marker(i,:) = rotationMatrixToVector(RotMatrix_ugvrs2marker(:,:,i));
    end
    RotVec_ugvrs2marker_mean = mean(RotVec_ugvrs2marker,1);
    RotMat_ugvrs2marker_mean = rotationVectorToMatrix(RotVec_ugvrs2marker_mean);
    
    %Marker-RSCOL間位置姿勢の計算
    TransVec_marker2ugvrs_mean = - TransVec_ugvrs2marker_mean * RotMat_ugvrs2marker_mean.';
    RotMat_marker2ugvrs_mean = RotMat_ugvrs2marker_mean.';
    
    %MK-RSDPT間の位置姿勢計算
    RT_mk2rscol = zeros(4,4);
    RT_mk2rscol(4,1:3) = TransVec_marker2ugvrs_mean;
    RT_mk2rscol(1:3,1:3) = RotMat_marker2ugvrs_mean;
    RT_mk2rscol(4,4) = 1.0;
    RT_mk2rsdpt = RT_mk2rscol * rs0_RTcol2dpt;
    TransVec_marker2ugvrsdpt_mean = RT_mk2rsdpt(4,1:3);
    RotMat_marker2ugvrsdpt_mean = RT_mk2rsdpt(1:3,1:3);
    
    %結果の保存
    save poseparams.mat TransVec_marker2ugvrsdpt_mean RotMat_marker2ugvrsdpt_mean

end