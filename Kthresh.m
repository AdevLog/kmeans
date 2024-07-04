% Step 1: 
% Assign each data point to its nearest seed, and accordingly update the Kclusters.
% 
% Step 2: 
% Average the data vectors in each cluster to update the cluster center.


clc
clear all
close all
%% Load Image
Img = im2double(imread('flowers.jfif'));              % Load Image height*weight*3 double, range 0~1 
resImg = reshape(Img,size(Img,1)*size(Img,2),3);        % Color Features image size(h*w)*3
%% K-means
Seg = input('Enter number of segments: ');              % Cluster Numbers
Seeds = resImg(randi(size(resImg,1),Seg,1) ,:);         % Random seeds, range: 1~image size
colDL = zeros(size(resImg,1),Seg+2);                    % Distances and Labels, image size*(Seg+2) array
Iters = 20;                                             % K-means Iteration
thres = 1;

%while thres > 0.01
for n = 1:Iters
%     if Iters > 20 
%         break
%     end

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
        %Seeds(i,:) = mean(resImg(idx,:));                   % Average the data vectors
        Seeds(i,:) = mean(resImg(idx,:));
%         thres = norm(Seeds(i,:) - Seeds(i,:));
%         disp('thres = ');
%         disp(thres);

    end
    %Iters = Iters + 1;

    if n == 1
        X1 = zeros(size(resImg));
        for i = 1:size(resImg,1)
            idx = colDL(i,Seg+2);
            X1(i,:) = Seeds(idx,:);
        end
        T1 = reshape(X1,size(Img,1),size(Img,2),3);
    end

    if n == 10
        X2 = zeros(size(resImg));
        for i = 1:size(resImg,1)
            idx = colDL(i,Seg+2);
            X2(i,:) = Seeds(idx,:);
        end
        T2 = reshape(X2,size(Img,1),size(Img,2),3);
    end

    if n == 20
        X3 = zeros(size(resImg));
        for i = 1:size(resImg,1)
            idx = colDL(i,Seg+2);
            X3(i,:) = Seeds(idx,:);
        end
        T3 = reshape(X3,size(Img,1),size(Img,2),3);
    end    

end

% X = zeros(size(resImg));
% for i = 1:size(resImg,1)
%     idx = colDL(i,Seg+2);
%     X(i,:) = newSeeds(idx,:);
% end
% T = reshape(X,size(Img,1),size(Img,2),3);
%% Show
figure();
subplot(2,2,1);
imshow(Img);
title('original');

% subplot(1,2,2);
% imshow(T);
% title('segmented');

subplot(2,2,2);
imshow(T1);
title('Iterations 1');

subplot(2,2,3);
imshow(T2);
title('Iterations 10');

subplot(2,2,4);
imshow(T3);
title('Iterations 20');

disp('Iterations = ');
disp(Iters);