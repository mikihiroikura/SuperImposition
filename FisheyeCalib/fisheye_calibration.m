function fisheye_calibration()
%fusheye_calibration 魚眼レンズのCalibration
%   
    %動画からFrameを保存する
    load setup.mat video_dir fish_step squareSize fishparamfile
    vidObj = VideoReader(video_dir);
    allFrame = read(vidObj); %すべてのFrameを読み取る
    
    %チェッカーボード検出
    calibimg = allFrame(:,:,:,1:fish_step:size(allFrame,4));
    [imagePoints,boardSize] = detectCheckerboardPoints(calibimg);
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);

    %Calibration
    imageSize = [size(allFrame, 1), size(allFrame, 2)];
    fisheyeParams = estimateFisheyeParameters(imagePoints, worldPoints, imageSize);

    %Camera parameterの保存(MATLABプログラム用)
    save fishparams.mat fisheyeParams
    
    %Camera parameterのCSVへの保存(C++のプログラム用)
    fid = fopen(fishparamfile,'w');
    fprintf(fid,'%.15f,',fisheyeParams.Intrinsics.MappingCoefficients);
    fprintf(fid,'\n');
    fprintf(fid,'%f,',fisheyeParams.Intrinsics.StretchMatrix);
    fprintf(fid,'\n');
    fprintf(fid,'%f,',fisheyeParams.Intrinsics.DistortionCenter);
    fprintf(fid,'\n');
    fprintf(fid,'%f,',fisheyeParams.RotationMatrices(:,:,1));
    fprintf(fid,'\n');
    fclose(fid);
end

