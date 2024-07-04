% Step 1: 
% Assign each data point to its nearest seed, and accordingly update the Kclusters.
% 
% Step 2: 
% Average the data vectors in each cluster to update the cluster center.


clc
clear all
close all
%% Load Image
Img = im2double(imread('house.jpg'));                   % Load Image, h*w*3 double, range 0~1 
resImg = reshape(Img,size(Img,1)*size(Img,2),3);        % Color Features, image size(h*w)*3
%% Color Table
SegColor = [0 0 1; 0 0 0; 1 0 0; 0 1 0; 1 1 0; 0 1 1; 1 0 1; 1 1 1];
%% K-means
Seg = input('Enter number of segments: ');              % Cluster Numbers
Seeds = resImg(randi(size(resImg,1),Seg,1) ,:);         % Random seeds, range: 1~image size
colDL = zeros(size(resImg,1),Seg+2);                    % Distances and Labels, image size*(Seg+2) array
Iters = 20;                                             % K-means Iteration

for n = 1:Iters

    % Distance between data vectors and centers
    for i = 1:size(resImg,1)
        for j = 1:Seg  
            colDL(i,j) = norm(resImg(i,:) - Seeds(j,:));    % Store distance in colDL
        end
        [minDistance, Lable] = min(colDL(i,1:Seg));
        colDL(i,Seg+1) = minDistance;                       % Seg+1 is Minimum Distance
        colDL(i,Seg+2) = Lable;                             % Seg+2 is Cluster Label
    end

    % New cluster centers
    for i = 1:Seg
        idx = find(colDL(:,Seg+2) == i);                    % Find index in Lable i
        Seeds(i,:) = mean(resImg(idx,:));                   % Average the data vectors
    end

end
%% Store Image
X = zeros(size(resImg));
for i = 1:size(resImg,1)
    idx = colDL(i,Seg+2);
    %X(i,:) = Seeds(idx,:);    
    X(i,:) = SegColor(idx,:);                               
end
T = reshape(X,size(Img,1),size(Img,2),3);
%% Show
figure();
subplot(1,2,1);
imshow(Img);
title('original');

subplot(1,2,2);
imshow(T);
title('segmented');