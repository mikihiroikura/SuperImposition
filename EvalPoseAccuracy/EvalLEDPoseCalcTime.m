%初めにCSVを読み込む

%検出失敗時の計算時間平均
time_faults = LEDposeresults.VarName3(find(LEDposeresults.VarName2~=0)+1);
time_faults_means = mean(time_faults);

%検出成功時の計算時間平均
X = find(LEDposeresults.VarName2==0)+1;
X(end) = [];
X = [X;1];
time_success = LEDposeresults.VarName3(X);
time_success_mean = mean(time_success);

%全体の計算時間平均
total_mean = mean(LEDposeresults.VarName3);