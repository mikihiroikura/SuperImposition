function RS_calibration()
    load setup.mat rs0_video_dir rs1_video_dir fish_step squareSize
    
    %RS0からFrameを保存する
    rs0vidObj = VideoReader(rs0_video_dir);
    allFrame = read(rs0vidObj);
    
    %チェッカーボード検出
    calibimg = allFrame(:,:,:,1:fish_step:size(allFrame,4));
    [imagePoints,boardSize] = detectCheckerboardPoints(calibimg);
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    
    %Calibration
    imageSize = [size(allFrame, 1), size(allFrame, 2)];
    rs0params = estimateCameraParameters(imagePoints,worldPoints, ...
                                  'ImageSize',imageSize);
                              
    %RS0 parameterの保存(MATLAB用)
    save rs0params.mat rs0params
    
    
    %RS1からFrameを保存する
    rs1vidObj = VideoReader(rs1_video_dir);
    allFrame = read(rs1vidObj);
    
    %チェッカーボード検出
    calibimg = allFrame(:,:,:,1:fish_step:size(allFrame,4));
    [imagePoints,boardSize] = detectCheckerboardPoints(calibimg);
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    
    %Calibration
    imageSize = [size(allFrame, 1), size(allFrame, 2)];
    rs1params = estimateCameraParameters(imagePoints,worldPoints, ...
                                  'ImageSize',imageSize);
                              
    %RS0 parameterの保存(MATLAB用)
    save rs1params.mat rs1params
    
end