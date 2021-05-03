function make_setup()
%make_setup() 最初にCalibrationする際の必要パラメータの設定
%   
    %魚眼カメラのCalibration用変数
    %fisheye_calibration()
    video_folder = './videos/fisheye/';
    video_name = '202105011642_video_hsc.mp4';
    video_dir = strcat(video_folder, video_name);
    fish_step = 10;
    squareSize = 32;
    form = 'yyyymmddHHMM';
    fishparamfile = strcat('./calib_result/fisheye/',strcat(datestr(now,form),'_fisheyeparam.csv'));
    poseparamfile = strcat('./calib_result/pose/',strcat(datestr(now,form),'_poseparam.csv'));
    
    %RSのカメラのCalibration用変数
    %RS_calibration()
    %RS0：UGV側
    rs0_video_folder = './videos/RS0/';
    rs0_video_name = '202105011540_video_rs0.mp4';
    rs0_video_dir = strcat(rs0_video_folder, rs0_video_name);
    %RS1：UAV側
    rs1_video_folder = './videos/RS1/';
    rs1_video_name = '202105011544_video_rs1.mp4';
    rs1_video_dir = strcat(rs1_video_folder, rs1_video_name);
    
    %共通パラメータ
    img_step = 20;
    
    %UAV側のRS-HSC間の位置姿勢Calibration
    video_folder_uav = './videos/UAV/';
    video_name_uavcalib_uavrs = 'UAVRS/202105011912_video_rs1.mp4';
    video_dir_uavcalib_uavrs = strcat(video_folder_uav, video_name_uavcalib_uavrs);
    video_name_uavcalib_ugvrs = 'UGVRS/202105011912_video_rs0.mp4';
    video_dir_uavcalib_ugvrs = strcat(video_folder_uav, video_name_uavcalib_ugvrs);
    csv_name_uavcalib = 'CSV/202105011912_markerpose.csv';
    csv_dir_uavcalib_marker = strcat(video_folder_uav, csv_name_uavcalib);
    
    %UGV側のRS-Marker間の位置姿勢Calibration
    video_folder_ugv = './videos/UGV/';
    video_name_ugv_rs = 'RS/202105011905_video_rs0.mp4';
    video_dir_ugv_rs = strcat(video_folder_ugv, video_name_ugv_rs);
    video_name_ugv_hsc = 'HSC/202105011905_video_hsc.mp4';
    video_dir_ugv_hsc = strcat(video_folder_ugv, video_name_ugv_hsc);
    csv_name_ugv = 'CSV/202105011905_markerpose.csv';
    csv_dir_ugv_marker = strcat(video_folder_ugv, csv_name_ugv);
    time_margin = 3;
    
    save setup.mat video_dir fish_step squareSize fishparamfile video_dir_uavcalib_uavrs img_step ...
        video_dir_ugv_rs video_dir_ugv_hsc csv_dir_ugv_marker rs0_video_dir rs1_video_dir poseparamfile ...
        video_dir_uavcalib_ugvrs csv_dir_uavcalib_marker time_margin
end

