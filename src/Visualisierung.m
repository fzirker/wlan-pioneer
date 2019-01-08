%Array = csvread('bag-filesfiltered.csv');
Array = csvread('visualisierung.csv');
x = Array(:,1);
y = Array(:,2);
g24 = Array(:,3);
g5 = Array(:,4);
plot(x,y);

