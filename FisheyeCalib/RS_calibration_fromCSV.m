function RS_calibration_fromCSV()
    rs0_csv_dir = './videos/RS0/202105051710_RS0_intrinsicparams.csv';
    rs1_csv_dir = './videos/RS1/202105051710_RS1_intrinsicparams.csv';
    

    M_rs0 = csvread(rs0_csv_dir);
    M_rs1 = csvread(rs1_csv_dir);
    
    rs0_fl = [M_rs0(3,1) M_rs0(3,2)];
    rs0_pp = [M_rs0(2,1) M_rs0(2,2)];
    rs0_imgsize = [M_rs0(1,2) M_rs0(1,1)];
    rs0_rd = [M_rs0(4,1) M_rs0(4,2) M_rs0(4,5)];
    rs0_td = [M_rs0(4,3) M_rs0(4,4)];
    rs0params = cameraIntrinsics(rs0_fl, rs0_pp, rs0_imgsize, 'RadialDistortion', rs0_rd, 'TangentialDistortion', rs0_td);
    
    rs1_fl = [M_rs1(3,1) M_rs1(3,2)];
    rs1_pp = [M_rs1(2,1) M_rs1(2,2)];
    rs1_imgsize = [M_rs1(1,2) M_rs1(1,1)];
    rs1_rd = [M_rs1(4,1) M_rs1(4,2) M_rs1(4,5)];
    rs1_td = [M_rs1(4,3) M_rs1(4,4)];
    rs1params = cameraIntrinsics(rs1_fl, rs1_pp, rs1_imgsize, 'RadialDistortion', rs1_rd, 'TangentialDistortion', rs1_td);

    save rs0params.mat rs0params
    save rs1params.mat rs1params
    
end