function uav_rs2hsc_posecalibration_rev2()
    load rs0params.mat rs0params
    load rs1params.mat rs1params
    load setup.mat video_dir_uavcalib_uavrs video_dir_uavcalib_ugvrs ...
        csv_dir_uavcalib_marker img_step squareSize
    
    %UAVRSの動画
    %動画からFrameを保存する
    vidObj_uavrs = VideoReader(video_dir_uavcalib_uavrs);
    allFrame_uavrs = read(vidObj_uavrs);
    uavrs_img = allFrame_uavrs(:,:,:,1:img_step:end);
    
    %チェッカーボードを検出する
    [imagePoints_uavrs,boardSize,imagesUsed_uavrs] = detectCheckerboardPoints(uavrs_img);
    worldPoints_uavrs = generateCheckerboardPoints(boardSize, squareSize);
    
    %UGVRSの動画
    %動画からFrameを保存する
    vidObj_ugvrs = VideoReader(video_dir_uavcalib_ugvrs);
    allFrame_ugvrs = read(vidObj_ugvrs);
    ugvrs_img = allFrame_ugvrs(:,:,:,1:img_step:end);
    
    %チェッカーボードを検出する
    [imagePoints_ugvrs,boardSize,imagesUsed_ugvrs] = detectCheckerboardPoints(ugvrs_img);
    worldPoints_ugvrs = generateCheckerboardPoints(boardSize, squareSize);
    
    %CSVからMarkerの位置姿勢を読み取る
    M =csvread(csv_dir_uavcalib_marker);
    M_marker = M(1:img_step:end,:);
    TransVec_marker2hsc = M_marker(:,10:12) * 1000;%単位はmm
    RotMatrix_marker2hsc = zeros(3,3,size(M_marker,1));
    for i = 1:size(M_marker,1)
        RotMatrix_marker2hsc(:,:,i) = reshape(M_marker(i,1:9),[3 3]).';
    end
    
    %UAVRSとUGVRS,CSVで検出したフレームの洗い出し
    imageUsed_ugvrs_uavrs_marker = logical(imagesUsed_uavrs .* imagesUsed_ugvrs);
    cnt = 0;
    for i = 1:size(imagesUsed_uavrs,1)
        if imagesUsed_uavrs(i)
            cnt = cnt + 1;
            if size(find(isnan(imagePoints_uavrs(:,:,cnt))),1) > 0
               imageUsed_ugvrs_uavrs_marker(i) = 0; 
            end
        end
    end
    cnt = 0;
    for i = 1:size(imagesUsed_ugvrs,1)
        if imagesUsed_ugvrs(i)
            cnt = cnt + 1;
            if size(find(isnan(imagePoints_ugvrs(:,:,cnt))),1) > 0
               imageUsed_ugvrs_uavrs_marker(i) = 0; 
            end
        end
    end
    imageUsed_ugvrs_uavrs_marker(TransVec_marker2hsc(:,1)==0) = 0;
    
    %再度チェッカーボード検出
    uavrs_img_used = uavrs_img(:,:,:,imageUsed_ugvrs_uavrs_marker);
    ugvrs_img_used = ugvrs_img(:,:,:,imageUsed_ugvrs_uavrs_marker);
    [imagePoints_uavrs,boardSize,imagesUsed_uavrs] = detectCheckerboardPoints(uavrs_img_used);
    worldPoints_uavrs = generateCheckerboardPoints(boardSize, squareSize);
    [imagePoints_ugvrs,boardSize,imagesUsed_ugvrs] = detectCheckerboardPoints(ugvrs_img_used);
    worldPoints_ugvrs = generateCheckerboardPoints(boardSize, squareSize);
    
    %再度CSVからMarkerの位置姿勢を読み取る
    TransVec_marker2hsc = M_marker(imageUsed_ugvrs_uavrs_marker,10:12) * 1000;%単位はmm
    RotMatrix_marker2hsc = zeros(3,3,size(M_marker(imageUsed_ugvrs_uavrs_marker,:),1));
    M_used = M_marker(imageUsed_ugvrs_uavrs_marker,:);
    for i = 1:size(M_used,1)
        RotMatrix_marker2hsc(:,:,i) = reshape(M_used(i,1:9),[3 3]).';
    end
    
    %Marker-UGVRS間位置姿勢の呼び出し
    load poseparams.mat TransVec_marker2ugvrs_mean RotMat_marker2ugvrs_mean
    
    %チェッカーボード(World)toUAVRSの外部パラメータの計算
    RotMatrix_uavrs = zeros(3,3,size(imagePoints_uavrs,3));
    TransVec_uavrs = zeros(size(imagePoints_uavrs,3), 3);
    for i=1:size(imagePoints_uavrs,3)
        [R,t] = extrinsics(imagePoints_uavrs(:,:,i),worldPoints_uavrs,rs1params);
        RotMatrix_uavrs(:,:,i) = R;
        TransVec_uavrs(i,:) = t;
    end
    
    %チェッカーボード(World)toUGVRSの外部パラメータの計算
    RotMatrix_ugvrs = zeros(3,3,size(imagePoints_ugvrs,3));
    TransVec_ugvrs = zeros(size(imagePoints_ugvrs,3), 3);
    for i=1:size(imagePoints_ugvrs,3)
        [R,t] = extrinsics(imagePoints_ugvrs(:,:,i),worldPoints_ugvrs,rs0params);
        RotMatrix_ugvrs(:,:,i) = R;
        TransVec_ugvrs(i,:) = t;
    end
    
    %同次座標変換行列を作成
    RTcb2uavrs = zeros(4,4,size(RotMatrix_uavrs,3));
    RTcb2ugvrs = zeros(4,4,size(RotMatrix_ugvrs,3));
    RTmk2ugvrs = eye(4,4);
    RTmk2hsc = zeros(4,4,size(RotMatrix_marker2hsc,3));
    RTcb2uavrs(1:3,1:3,:) = RotMatrix_uavrs;
    RTcb2uavrs(4,1:3,:) = TransVec_uavrs.';
    RTcb2uavrs(4,4,:) = 1;
    RTcb2ugvrs(1:3,1:3,:) = RotMatrix_ugvrs;
    RTcb2ugvrs(4,1:3,:) = TransVec_ugvrs.';
    RTcb2ugvrs(4,4,:) = 1;
    RTmk2ugvrs(1:3,1:3) = RotMat_marker2ugvrs_mean;
    RTmk2ugvrs(4,1:3) = TransVec_marker2ugvrs_mean;
    RTmk2hsc(1:3,1:3,:) = RotMatrix_marker2hsc;
    RTmk2hsc(4,1:3,:) = TransVec_marker2hsc.';
    RTmk2hsc(4,4,:) = 1;
    
    %UAVRS-HSC間の変換行列を計算
    RTuavrs2hsc = zeros(4,4,size(RotMatrix_uavrs,3));
    for i = 1:size(RotMatrix_uavrs,3)
        RTuavrs2hsc(:,:,i) = inv(RTcb2uavrs(:,:,i)) * RTcb2ugvrs(:,:,i) * inv(RTmk2ugvrs) * RTmk2hsc(:,:,i);
    end
    
    %UAVRS-HSC間の並進ベクトルと回転行列の平均値を計算
    TransVec_uavrs2hsc = RTuavrs2hsc(4,1:3,:);
    RotMat_uavrs2hsc = RTuavrs2hsc(1:3,1:3,:);
    TransVec_uavrs2hsc_mean = mean(TransVec_uavrs2hsc,3);
    RotVec_uavrs2hsc = zeros(size(RTuavrs2hsc,3), 3);
    for i = 1:size(RotMat_uavrs2hsc,3)
        RotVec_uavrs2hsc(i,:) = rotationMatrixToVector(RotMat_uavrs2hsc(:,:,i));
    end
    RotVec_uavrs2hsc_mean = mean(RotVec_uavrs2hsc,1);
    RotMat_uavrs2hsc_mean = rotationVectorToMatrix(RotVec_uavrs2hsc_mean);

    %結果の保存
    save poseparams.mat TransVec_marker2ugvrs_mean RotMat_marker2ugvrs_mean TransVec_uavrs2hsc_mean RotMat_uavrs2hsc_mean
    
    %CSVへの出力
    load setup.mat poseparamfile
    fid = fopen(poseparamfile,'w');
    fprintf(fid,'%.6f,',RotMat_uavrs2hsc_mean);
    fprintf(fid,'\n');
    fprintf(fid,'%.6f,',TransVec_uavrs2hsc_mean);
    fprintf(fid,'\n');
    fprintf(fid,'%.6f,',RotMat_marker2ugvrs_mean);
    fprintf(fid,'\n');
    fprintf(fid,'%.6f,',TransVec_marker2ugvrs_mean);
    fprintf(fid,'\n');
    fclose(fid);
    
end