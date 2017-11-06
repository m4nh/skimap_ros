

kdtree = importfile('bench_points/kdtree.csv');
kdskip_1 = importfile('bench_points/kdskip_r1.csv');
kdskip_5 = importfile('bench_points/kdskip_r5.csv');
kdskip_10 = importfile('bench_points/kdskip_r10.csv');
kdskip_20 = importfile('bench_points/kdskip_r20.csv');


hold on;
c = 7;
plot(kdtree(:,c));
plot(kdskip_1(:,c));
plot(kdskip_5(:,c));
plot(kdskip_10(:,c));
plot(kdskip_20(:,c));
legend('kdtree','kdskip r1','kdskip r5','kdskip r10','kdskip r20');