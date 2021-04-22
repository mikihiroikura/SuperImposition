function uav_rs2hsc_posecalibration_rev2()
    load fishparams.mat fisheyeParams
    load rs0params.mat rs0params
    load rs1params.mat rs1params
    
    %UGVRSの動画
    %動画からFrameを保存する
    load setup.mat video_dir_uav_rs video_dir_uav_hsc img_step squareSize
    vidObj_uavrs = VideoReader(video_dir_uav_rs);
    allFrame_uavrs = read(vidObj_uavrs);
    uavrs_img = allFrame_uavrs(:,:,:,1:img_step:end);
end