%各種パラメータの読み取り
hscimg_csv_name = 'data/old/20210515_2025_500hz/HSCimg_times.csv';
ledpose_csv_name = 'data/old/20210515_2025_500hz/LEDpose_results.csv';
hsc_pngs = dir('data/old/20210515_2025_500hz/HSC/*.png');
calibratedpose_csv_name = '202105051723_poseparam.csv';
squareSize = 32;
hscwidth = 896;
hscheight = 896;
load fishparams.mat fisheyeParams
gethscimgstep = 15;

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

%指定したLEDマーカに時刻が最も近いHSC画像ID
hscids = [];
cnt = 1;
selectledtime = M_ledpose(1:gethscimgstep:end,1);
for i = 1:size(selectledtime,1)
    while selectledtime(i)>M_hsctime(cnt,1)
        cnt = cnt + 1;
    end
    if cnt~=1
        hscids = [hscids;cnt-1];
    else
        hscids = [hscids;cnt];
    end
end

%HSCの画像取得
hsc_imgs = uint8(zeros(hscheight, hscwidth, 3, size(hscids,1)));
for k = 1:size(hscids,1)
    hsc_imgs(:,:,:,k) = imread(strcat(hsc_pngs(hscids(k)).folder,strcat('\',hsc_pngs(hscids(k)).name)));
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

%使用するHSCのIDのみ
hscids = hscids(imageUsed_hsc);
ledids = 1:gethscimgstep:size(M_ledpose,1);
ledids = ledids.';
ledids = ledids(imageUsed_hsc);

%HSCからCBの検出，位置姿勢保存
worldPoints_hsc = generateCheckerboardPoints(boardSize_hsc, squareSize);
RT_cb2hsc = zeros(4,4,size(hsc_imgs_used,4));
for k = 1:size(hsc_imgs_used,4)
    [R,t] = extrinsics(imagePoints_hsc(:,:,k),worldPoints_hsc,fisheyeParams.Intrinsics);
    RT_cb2hsc(1:3,1:3,k) = R;
    RT_cb2hsc(4,1:3,k) = t.';
end
RT_cb2hsc(4,4,:) = 1.0;



%LEDposeから，計測したUAVRS2UGVRSを呼び出す
%RTugvmk2rs * RTc2m * RTuavrs2hsc;を意味する
ledtime = M_ledpose(ledids,1);
RT_uavrs2ugvrs_ledpose = zeros(4,4,size(ledtime,1));
RT_uavrs2ugvrs_ledpose(1:3,1:3,:) = reshape(M_ledpose(ledids,4:12)', 3,3,[]);
RT_uavrs2ugvrs_ledpose(4,1:3,:) = M_ledpose(ledids,13:15).'* 1000;
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
Tdiff_cb2mk = squeeze(RTdiff_cb2mk(4,1:3,:)).';
Tthrid = M_ledpose(ledids,2)==0;
Rvecthrid = M_ledpose(ledids,2)==0;
mean(Tdiff_cb2mk(Tthrid,:))
std(Tdiff_cb2mk(Tthrid,:))
mean(Rvecdiff_cb2mk(Rvecthrid,:))
std(Rvecdiff_cb2mk(Rvecthrid,:))