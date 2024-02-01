num = 10000;
Sigma1 = diag([4 9]/3); % 这儿为什么除3协方差就一样了
randomSamples1 = mvnrnd(zeros(2,1), Sigma1, num);
randomSamples1 = randomSamples1';
scatter(randomSamples1(1,:),randomSamples1(2,:),'*','b', 'LineWidth',1)
hold on
scatter(2*rands(num,1), 3*rands(num,1),'o','filled','g')