%%%　全体の動作を決定する関数
% Color

% 初期化
make_setup_rgb();
disp('Setup Finished.');

% 魚眼カメラのキャリブレーション
fisheye_calibration_rgb();
disp('Camera Calibration finished.');