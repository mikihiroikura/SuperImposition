%%%　全体の動作を決定する関数
% Color

% 初期化
make_setup();
disp('Setup Finished.');

% 魚眼カメラのキャリブレーション
fisheye_calibration();
disp('Camera Calibration finished.');

% 2つのRealSenseキャリブレーション
RS_calibration();
disp('RealSense Calibration finished.');

% UGVのRS-Marker間のキャリブレーション
ugv_rs2marker_posecalibration();
disp('UGVRS2Marker Calibration finished.');

% UAVのRS-HSC間のキャリブレーション
uav_rs2hsc_posecalibration_rev2();
disp('UAVRS2HSC Calibration finished.');