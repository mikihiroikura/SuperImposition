%各種パラメータの読み取り
hscimg_csv_name = 'data/HSCimg_times.csv';
ledpose_csv_name = 'data/LEDpose_results.csv';
calibratedpose_csv_name = '202105051723_poseparam.csv';
squareSize = 32;
hscwidth = 896;
hscheight = 896;
load fishparams.mat fisheyeParams
gethscimgstep = 10;

%CSV読み取り
M_ledpose = csvread(ledpose_csv_name);
M_hsctime = csvread(hscimg_csv_name);
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

%HSCの画像取得
hsc_pngs = dir('data/HSC/*.png');
hsc_imgs = uint8(zeros(hscheight, hscwidth, 3, size(1:gethscimgstep:size(hsc_pngs,1),2)));
cnt = 1;
for k = 1:gethscimgstep:length(hsc_pngs)
    hsc_imgs(:,:,:,cnt) = imread(strcat(hsc_pngs(k).folder,strcat('\',hsc_pngs(k).name)));
    cnt = cnt + 1;
end

%HSCから位置姿勢計算
[imagePoints_hsc,boardSize_hsc,imageUsed] = detectCheckerboardPoints(hsc_imgs);
%HSCで検出したフレームの洗い出し
imageUsed_hsc = imageUsed;
cnt = 0;
for i = 1:size(imageUsed,1)
    if imageUsed(i)
        cnt = cnt + 1;
        if size(find(isnan(imagePoints_hsc(:,:,cnt))),1) > 0
           imageUsed_hsc(i) = 0; 
        end
    end
end

%再度チェッカーボード検出
hsc_imgs_used = hsc_imgs(:,:,:,imageUsed_hsc);
[imagePoints_hsc,boardSize_hsc,imageUsed] = detectCheckerboardPoints(hsc_imgs_used);
M_hsctime_used = M_hsctime(imageUsed_hsc,:);

%HSCからCBの検出，位置姿勢保存
worldPoints_hsc = generateCheckerboardPoints(boardSize_hsc, squareSize);
RT_cb2hsc = zeros(4,4,size(hsc_imgs_used,4));
for k = 1:size(hsc_imgs_used,4)
    [R,t] = extrinsics(imagePoints_hsc(:,:,k),worldPoints_hsc,fisheyeParams.Intrinsics);
    RT_cb2hsc(1:3,1:3,k) = R;
    RT_cb2hsc(4,1:3,k) = t.';
end
RT_cb2hsc(4,4,:) = 1.0;

%指定したHSC画像に時刻が最も近いLEDマーカの位置姿勢ID
poseids = [];
cnt = 1;
hsctime = M_hsctime(1:gethscimgstep:end,1);
for i = 1:size(hsctime,1)
while hsctime(i)>M_ledpose(cnt,1)
cnt = cnt + 1;
end
poseids = [poseids;cnt];
end
poseids = poseids(imageUsed_hsc);

%LEDposeから，計測したUAVRS2UGVRSを呼び出す
%RTugvmk2rs * RTc2m * RTuavrs2hsc;を意味する
ledtime = M_ledpose(poseids,1);
RT_uavrs2ugvrs_ledpose = zeros(4,4,size(ledtime,1));
RT_uavrs2ugvrs_ledpose(1:3,1:3,:) = reshape(M_ledpose(poseids,4:12)', 3,3,[]);
RT_uavrs2ugvrs_ledpose(4,1:3,:) = M_ledpose(poseids,13:15).'* 1000;
RT_uavrs2ugvrs_ledpose(4,4,:) = 1.0;

%HSC2MKの位置姿勢を計算する
RT_hsc2mk = zeros(size(RT_uavrs2ugvrs_ledpose));
for k = 1:size(RT_hsc2mk, 3)
    if RT_uavrs2ugvrs_ledpose(4,1,k)~=0
        RT_hsc2mk(:,:,k) = RT_uavrs2hsc(:,:) \ (RT_uavrs2ugvrs_ledpose(:,:,k)) / RT_mk2ugvrs(:,:);
    end
end

%CB2MKの変換行列を求める
RT_cb2mk = zeros(size(RT_hsc2mk));
hsctimeid = 1;
for i = 1:size(RT_cb2mk,3)
    if RT_hsc2mk(4,1,i)~=0 && RT_cb2hsc(4,1,i)~=0
        RT_cb2mk(:,:,i) = RT_cb2hsc(:,:,i) * RT_hsc2mk(:,:,i);
    end
end

%CB2MKの変動を計算
RTdiff_cb2mk = zeros(size(RT_cb2mk));
Rvecdiff_cb2mk = zeros(size(RT_cb2mk,3),3);
for i=1:size(RT_cb2mk,3)
    RTdiff_cb2mk(:,:,i) = RT_cb2mk(:,:,1) / RT_cb2mk(:,:,i);
    Rvecdiff_cb2mk(i,:) = rotationMatrixToVector(RTdiff_cb2mk(1:3,1:3,i)) * 180 /pi;
end

%位置変動出力
figure
for k = 1:3
    plot(ledtime,squeeze(RTdiff_cb2mk(4,k,:)));
    hold on
end

%姿勢変動出力
figure
for k = 1:3
    plot(ledtime,(Rvecdiff_cb2mk(:,k)));
    hold on
end

%ずれの平均分散を計算
Tthr = 50;
Rvecthr = 10;
Tdiff_cb2mk = squeeze(RTdiff_cb2mk(4,1:3,:)).';
Tthrid = Tdiff_cb2mk(:,1)<Tthr & Tdiff_cb2mk(:,1)>-Tthr ...
    & Tdiff_cb2mk(:,2)<Tthr & Tdiff_cb2mk(:,2)>-Tthr ...
    & Tdiff_cb2mk(:,3)<Tthr & Tdiff_cb2mk(:,3)>-Tthr ...
    & M_ledpose(poseids,2)==0;
Rvecthrid = Rvecdiff_cb2mk(:,1)<Rvecthr & Rvecdiff_cb2mk(:,1)>-Rvecthr ...
    & Rvecdiff_cb2mk(:,2)<Rvecthr & Rvecdiff_cb2mk(:,2)>-Rvecthr ...
    & Rvecdiff_cb2mk(:,3)<Rvecthr & Rvecdiff_cb2mk(:,3)>-Rvecthr ...
    & M_ledpose(poseids,2)==0;
mean(Tdiff_cb2mk(Tthrid,:))
std(Tdiff_cb2mk(Tthrid,:))
mean(Rvecdiff_cb2mk(Rvecthrid,:))
std(Rvecdiff_cb2mk(Rvecthrid,:))