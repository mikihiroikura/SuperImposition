function RS_calibration()
    load setup.mat rs0_video_dir rs1_video_dir img_step squareSize
    
    %RS0からFrameを保存する
    rs0vidObj = VideoReader(rs0_video_dir);
    allFrame_rs0 = read(rs0vidObj);
    
    %チェッカーボード検出
    calibimg_rs0 = allFrame_rs0(:,:,:,1:img_step*2:size(allFrame_rs0,4));
    [imagePoints_rs0,boardSize_rs0] = detectCheckerboardPoints(calibimg_rs0);
    worldPoints_rs0 = generateCheckerboardPoints(boardSize_rs0, squareSize);
    
    %Calibration
    imageSize_rs0 = [size(allFrame_rs0, 1), size(allFrame_rs0, 2)];
    rs0params = estimateCameraParameters(imagePoints_rs0,worldPoints_rs0, ...
                                  'ImageSize',imageSize_rs0, 'EstimateSkew', true, 'NumRadialDistortionCoefficients', 3);
                               
    %RS1からFrameを保存する
    rs1vidObj = VideoReader(rs1_video_dir);
    allFrame_rs1 = read(rs1vidObj);
    
    %チェッカーボード検出
    calibimg_rs1 = allFrame_rs1(:,:,:,1:img_step*2:size(allFrame_rs1,4));
    [imagePoints_rs1,boardSize_rs1] = detectCheckerboardPoints(calibimg_rs1);
    worldPoints_rs1 = generateCheckerboardPoints(boardSize_rs1, squareSize);
    
    %Calibration
    imageSize_rs1 = [size(allFrame_rs1, 1), size(allFrame_rs1, 2)];
    rs1params = estimateCameraParameters(imagePoints_rs1,worldPoints_rs1, ...
                                  'ImageSize',imageSize_rs1, 'EstimateSkew', true, 'NumRadialDistortionCoefficients', 3);
    
    %RS0 parameterの保存(MATLAB用)
    save rs0params.mat rs0params
    %RS1 parameterの保存(MATLAB用)
    save rs1params.mat rs1params
    
end