function make_setup()
%make_setup() 最初にCalibrationする際の必要パラメータの設定
%   
    %魚眼カメラのCalibration用変数
    %fisheye_calibration()
    video_folder = './videos/fisheye/';
    video_name = '202104191516_video_hsc.mp4';
    video_dir = strcat(video_folder, video_name);
    fish_step = 10;
    squareSize = 32;
    form = 'yyyymmddHHMM';
    fishparamfile = strcat('./calib_result/fisheye/',strcat(datestr(now,form),'_fisheyeparam.csv'));
    
    %RSのカメラのCalibration用変数
    %RS_calibration()
    %RS0：UGV側
    rs0_video_folder = './videos/RS0/';
    rs0_video_name = '202104191510_video_rs0.mp4';
    rs0_video_dir = strcat(rs0_video_folder, rs0_video_name);
    %RS1：UAV側
    rs1_video_folder = './videos/RS1/';
    rs1_video_name = '202104191507_video_rs1.mp4';
    rs1_video_dir = strcat(rs1_video_folder, rs1_video_name);
    
    %共通パラメータ
    img_step = 20;
    
    %UAV側のRS-HSC間の位置姿勢Calibration
    video_folder_uav = './videos/UAV/';
    video_name_uav_rs = 'RS/202104191536_video_rs1.mp4';
    video_dir_uav_rs = strcat(video_folder_uav, video_name_uav_rs);
    video_name_uav_hsc = 'HSC/202104191536_video_hsc.mp4';
    video_dir_uav_hsc = strcat(video_folder_uav, video_name_uav_hsc);
    
    %UGV側のRS-Marker間の位置姿勢Calibration
    video_folder_ugv = './videos/UGV/';
    video_name_ugv_rs = 'RS/202104191900_video_rs0.mp4';
    video_dir_ugv_rs = strcat(video_folder_ugv, video_name_ugv_rs);
    video_name_ugv_hsc = 'HSC/202104191900_video_hsc.mp4';
    video_dir_ugv_hsc = strcat(video_folder_ugv, video_name_ugv_hsc);
    csv_name_ugv = 'CSV/202104191900_markerpose.csv';
    csv_dir_ugv_marker = strcat(video_folder_ugv, csv_name_ugv);
    
    save setup.mat video_dir fish_step squareSize fishparamfile video_dir_uav_rs video_dir_uav_hsc img_step ...
        video_dir_ugv_rs video_dir_ugv_hsc csv_dir_ugv_marker rs0_video_dir rs1_video_dir
end

