function uav_rs2hsc_posecalibration_rev2()
    load rs0params.mat rs0params rs0_RTcol2dpt
    load rs1params.mat rs1params rs1_RTcol2dpt
    load setup.mat video_dir_uavcalib_uavrs video_dir_uavcalib_ugvrs ...
        csv_dir_uavcalib_marker img_step squareSize time_margin
    
    %UAVRSの動画
    %動画からFrameを保存する
    vidObj_uavrs = VideoReader(video_dir_uavcalib_uavrs);
    allFrame_uavrs = read(vidObj_uavrs);
    uavrs_img = allFrame_uavrs(:,:,:,int16(time_margin*vidObj_uavrs.FrameRate):img_step:int16((vidObj_uavrs.Duration-time_margin)*vidObj_uavrs.FrameRate));
    
    %チェッカーボードを検出する
    [imagePoints_uavrs,boardSize,imagesUsed_uavrs] = detectCheckerboardPoints(uavrs_img);
    worldPoints_uavrs = generateCheckerboardPoints(boardSize, squareSize);
    
    %UGVRSの動画
    %動画からFrameを保存する
    vidObj_ugvrs = VideoReader(video_dir_uavcalib_ugvrs);
    allFrame_ugvrs = read(vidObj_ugvrs);
    ugvrs_img = allFrame_ugvrs(:,:,:,int16(time_margin*vidObj_ugvrs.FrameRate):img_step:int16((vidObj_ugvrs.Duration-time_margin)*vidObj_ugvrs.FrameRate));
    
    %チェッカーボードを検出する
    [imagePoints_ugvrs,boardSize,imagesUsed_ugvrs] = detectCheckerboardPoints(ugvrs_img);
    worldPoints_ugvrs = generateCheckerboardPoints(boardSize, squareSize);
    
    %CSVからMarkerの位置姿勢を読み取る
    M =csvread(csv_dir_uavcalib_marker);
    M_marker = M(int16(time_margin*vidObj_ugvrs.FrameRate):img_step:int16((vidObj_ugvrs.Duration-time_margin)*vidObj_ugvrs.FrameRate),:);
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
    load poseparams.mat TransVec_marker2ugvrsdpt_mean RotMat_marker2ugvrsdpt_mean
    
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
    RTmk2ugvrsdpt = eye(4,4);
    RTmk2hsc = zeros(4,4,size(RotMatrix_marker2hsc,3));
    RTcb2uavrs(1:3,1:3,:) = RotMatrix_uavrs;
    RTcb2uavrs(4,1:3,:) = TransVec_uavrs.';
    RTcb2uavrs(4,4,:) = 1;
    RTcb2ugvrs(1:3,1:3,:) = RotMatrix_ugvrs;
    RTcb2ugvrs(4,1:3,:) = TransVec_ugvrs.';
    RTcb2ugvrs(4,4,:) = 1;
    RTmk2ugvrsdpt(1:3,1:3) = RotMat_marker2ugvrsdpt_mean;
    RTmk2ugvrsdpt(4,1:3) = TransVec_marker2ugvrsdpt_mean;
    RTmk2hsc(1:3,1:3,:) = RotMatrix_marker2hsc;
    RTmk2hsc(4,1:3,:) = TransVec_marker2hsc.';
    RTmk2hsc(4,4,:) = 1;
    
    %RS関連同次座標変換行列をCOLからDPTに変換
    RTcb2uavrsdpt = pagemtimes(RTcb2uavrs, rs1_RTcol2dpt);
    RTcb2ugvrsdpt = pagemtimes(RTcb2ugvrs, rs0_RTcol2dpt);
    
    %UAVRS-HSC間の変換行列を計算
    RTuavrsdpt2hsc = zeros(4,4,size(RotMatrix_uavrs,3));
    for i = 1:size(RotMatrix_uavrs,3)
        RTuavrsdpt2hsc(:,:,i) = inv(RTcb2uavrsdpt(:,:,i)) * RTcb2ugvrsdpt(:,:,i) * inv(RTmk2ugvrsdpt) * RTmk2hsc(:,:,i);
    end
    
    %UAVRS-HSC間の並進ベクトルと回転行列の平均値を計算
    TransVec_uavrsdpt2hsc = squeeze(RTuavrsdpt2hsc(4,1:3,:)).';
    RotMat_uavrsdpt2hsc = RTuavrsdpt2hsc(1:3,1:3,:);
    TransVec_uavrsdpt2hsc_mean = mean(TransVec_uavrsdpt2hsc,1);
    RotVec_uavrsdpt2hsc = zeros(size(RTuavrsdpt2hsc,3), 3);
    for i = 1:size(RotMat_uavrsdpt2hsc,3)
        RotVec_uavrsdpt2hsc(i,:) = rotationMatrixToVector(RotMat_uavrsdpt2hsc(:,:,i));
    end
    RotVec_uavrsdpt2hsc_mean = mean(RotVec_uavrsdpt2hsc,1);
    RotMat_uavrsdpt2hsc_mean = rotationVectorToMatrix(RotVec_uavrsdpt2hsc_mean);
    
    %結果の保存
    save poseparams.mat TransVec_marker2ugvrsdpt_mean RotMat_marker2ugvrsdpt_mean TransVec_uavrsdpt2hsc_mean RotMat_uavrsdpt2hsc_mean
    
    %CSVへの出力
    load setup.mat poseparamfile
    fid = fopen(poseparamfile,'w');
    fprintf(fid,'%.6f,',RotMat_uavrsdpt2hsc_mean);
    fprintf(fid,'\n');
    fprintf(fid,'%.6f,',TransVec_uavrsdpt2hsc_mean);
    fprintf(fid,'\n');
    fprintf(fid,'%.6f,',RotMat_marker2ugvrsdpt_mean);
    fprintf(fid,'\n');
    fprintf(fid,'%.6f,',TransVec_marker2ugvrsdpt_mean);
    fprintf(fid,'\n');
    fclose(fid);
    
end