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
%Img = im2double(imread('finger.jfif'));
resImg = reshape(Img,size(Img,1)*size(Img,2),3);        % Color Features, image size(h*w)*3
%% Color Table
SegColor = [0 0 1; 0 0 0; 1 0 0; 0 1 0; 1 1 0; 0 1 1; 1 0 1; 1 1 1];
%% K-means
Seg = input('Enter number of segments: ');              % Cluster Numbers
Seeds = resImg(randi(size(resImg,1),Seg,1),:);          % Random seeds, range: 1~image size
resImgL = zeros(size(resImg,1),Seg+1);                  % Add Labels, image size*(Seg+1) array
Iters = 20;                                             % K-means Iteration

for n = 1:Iters

    % Distance between data vectors and centers
    for i = 1:size(resImg,1)
        for j = 1:Seg  
            resImgL(i,j) = norm(resImg(i,:) - Seeds(j,:));    % Store distance in resImgL
        end
        [minDistance, Lable] = min(resImgL(i,1:Seg));
        resImgL(i,Seg+1) = Lable;                             % Seg+1 is Cluster Label

    end

    % New cluster centers
    for i = 1:Seg
        idx = find(resImgL(:,Seg+1) == i);                    % Find index in Lable i
        Seeds(i,:) = mean(resImg(idx,:));                     % Average the data vectors
    end

end
%% Store Image
X = zeros(size(resImg));
for i = 1:size(resImg,1)
    idx = resImgL(i,Seg+1);                 %第i列 Seg+1是第幾類
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