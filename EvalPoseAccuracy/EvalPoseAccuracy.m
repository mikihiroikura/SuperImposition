%各種パラメータの読み取り
rsimg_csv_name = 'data/RSimg_times.csv';
ledpose_csv_name = 'data/LEDpose_results.csv';
calibratedpose_csv_name = '202105011946_poseparam.csv';
squareSize = 32;
rsimgwidth = 1920;
rsimgheight = 1080;
load rs0params.mat rs0params
load rs1params.mat rs1params

%CSV読み取り
M_rstime = csvread(rsimg_csv_name);
M_ledpose = csvread(ledpose_csv_name);
M_calibpose = csvread(calibratedpose_csv_name);


%Calibration済のデータの読み取り
RT_uavrs2hsc = zeros(4,4);
RT_uavrs2hsc(1:3,1:3) = reshape(M_calibpose(1,1:9),[3 3]);
RT_uavrs2hsc(4,1:3) = M_calibpose(2,1:3);
RT_uavrs2hsc(4,4) = 1.0;

RT_mk2ugvrs = zeros(4,4);
RT_mk2ugvrs(1:3,1:3) = reshape(M_calibpose(3,1:9),[3 3]);
RT_mk2ugvrs(4,1:3) = M_calibpose(4,1:3);
RT_mk2ugvrs(4,4) = 1.0;

%RSの画像取得
RS0_pngs = dir('data/RS0/*.png');
RS0_imgs = uint8(zeros(rsimgheight, rsimgwidth, 3, size(M_rstime,1)));
for k = 1:length(RS0_pngs)
    RS0_imgs(:,:,:,k) = imread(strcat(RS0_pngs(k).folder,strcat('\',RS0_pngs(k).name)));
end

RS1_pngs = dir('data/RS1/*.png');
RS1_imgs = uint8(zeros(rsimgheight, rsimgwidth, 3, size(M_rstime,1)));
for k = 1:length(RS1_pngs)
    RS1_imgs(:,:,:,k) = imread(strcat(RS1_pngs(k).folder,strcat('\',RS1_pngs(k).name)));
end

%RSから位置姿勢計算
[imagePoints_rs0,boardSize_rs0] = detectCheckerboardPoints(RS0_imgs);
worldPoints_rs0 = generateCheckerboardPoints(boardSize_rs0, squareSize);
imageSize_rs0 = [size(RS0_imgs, 1), size(RS0_imgs, 2)];
RT_cb2ugvrs = zeros(4,4,size(RS0_imgs,4));
for k = 1:size(RS0_imgs,4)
    [R,t] = extrinsics(imagePoints_rs0(:,:,k),worldPoints_rs0,rs0params);
    RT_cb2ugvrs(1:3,1:3,k) = R;
    RT_cb2ugvrs(4,1:3,k) = t.';
end
RT_cb2ugvrs(4,4,:) = 1.0;

[imagePoints_rs1,boardSize_rs1] = detectCheckerboardPoints(RS1_imgs);
worldPoints_rs1 = generateCheckerboardPoints(boardSize_rs1, squareSize);
imageSize_rs1 = [size(RS1_imgs, 1), size(RS1_imgs, 2)];
RT_cb2uavrs = zeros(4,4,size(RS1_imgs,4));
for k = 1:size(RS1_imgs,4)
    [R,t] = extrinsics(imagePoints_rs1(:,:,k),worldPoints_rs1,rs1params);
    RT_cb2uavrs(1:3,1:3,k) = R;
    RT_cb2uavrs(4,1:3,k) = t.';
end
RT_cb2uavrs(4,4,:) = 1.0;

%CB画像群からUAVRS2UGVRSの位置姿勢を計算する
RT_uavrs2ugvrs_cbimgs = zeros(size(RT_cb2uavrs));
for k = 1:size(RT_cb2uavrs,3)
    RT_uavrs2ugvrs_cbimgs(:,:,k) = RT_cb2uavrs(:,:,k) \ RT_cb2ugvrs(:,:,k);
end

%LEDposeから，計測したUAVRS2UGVRSを呼び出す
ledtime = M_ledpose(:,1);
RT_uavrs2ugvrs_ledpose = zeros(4,4,size(M_ledpose,1));
RT_uavrs2ugvrs_ledpose(1:3,1:3,:) = reshape(M_ledpose(:,4:12)', 3,3,[]);
RT_uavrs2ugvrs_ledpose(4,1:3,:) = M_ledpose(:,13:15).'* 1000;
RT_uavrs2ugvrs_ledpose(4,4,:) = 1.0;

%差分同次座標変換行列
RT_uavrs2ugvrs_diff = zeros(size(RT_uavrs2ugvrs_ledpose));
rstimeid = 1;
for i = 1:size(RT_uavrs2ugvrs_diff,3)
    while M_rstime(rstimeid)<ledtime(i)
        rstimeid = rstimeid + 1;
    end
    RT_uavrs2ugvrs_diff(:,:,i) = RT_uavrs2ugvrs_ledpose(:,:,i) / RT_uavrs2ugvrs_cbimgs(:,:,rstimeid);
end

%グラフで表現する
%並進ベクトルのずれ
for k = 1:3
    figure
    plot(ledtime,squeeze(RT_uavrs2ugvrs_ledpose(4,k,:)));
    hold on
    plot(M_rstime, squeeze(RT_uavrs2ugvrs_cbimgs(4,k,:)));
end

%回転ベクトルのずれ
%差分回転行列から差分角度を計算
Rvec_uavrs2ugvrs_diff = zeros(size(RT_uavrs2ugvrs_diff,3),3);
for i = 1:size(RT_uavrs2ugvrs_diff,3)
    Rvec_uavrs2ugvrs_diff(i,:) = -rotationMatrixToVector(RT_uavrs2ugvrs_diff(1:3,1:3,i));
end
for k = 1:3
    figure
    plot(ledtime,Rvec_uavrs2ugvrs_diff(:,k));
end

%ずれの平均分散を計算
% mean(RT_uavrs2ugvrs_diff(4,1,:))
% mean(RT_uavrs2ugvrs_diff(4,2,:))
% mean(RT_uavrs2ugvrs_diff(4,3,:))
% std(RT_uavrs2ugvrs_diff(4,1,:))
% std(RT_uavrs2ugvrs_diff(4,2,:))
% std(RT_uavrs2ugvrs_diff(4,3,:))


