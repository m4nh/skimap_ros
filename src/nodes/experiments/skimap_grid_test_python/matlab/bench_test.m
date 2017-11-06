d1 = importfile('skigrid_single.csv');
d2 = importfile('skigrid_omp.csv');

hold on;
c = 4;
plot(d1(:,c));
plot(d2(:,c));
legend('single','omp');