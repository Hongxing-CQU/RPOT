close all
clc
clear;  
load data.mat

[pointCountX,x_dim] = size(X);
[pointCountY,y_dim] = size(Y);
Mx = 1 / pointCountX * ones(pointCountX,1);
My = 1 / pointCountY * ones(pointCountY,1);

% algorithm params
para.epsilon = 0.004;
para.alpha = 0;
para.beta = 1;
para.alpha_totalmass =0; 
para.beta_totalmass =0.8;
para.threhold = 1e-5;

% init_epsilon = para.epsilon;
Xorg = X; Yorg = Y;

if x_dim == 2
    para.AnnealRate = 0.8;
else
    para.AnnealRate = 0.9; 
end    

common_plot(X, Y, 'BeforeRegistration');

% preprocess data, align the mass barycenter between X and Y
XmassBarycenter = sum(1 / pointCountX * X);
YmassBarycenter = sum(1 / pointCountY * Y);

X = bsxfun(@minus, X, XmassBarycenter);
Y = bsxfun(@minus, Y, YmassBarycenter);

tic
% compute transport distance matrix between X and Y
D = pdist2(X,Y,'squaredeuclidean');

disp('####### Partial OT Rigistration Start #######');    
[R0, t0, Ytransformed, D, T, para] = unbalanced_OT(X, Y, Mx, My, D, para);
disp('####### Partial OT Rigistration Finish #######');    

t0 = t0 + XmassBarycenter - YmassBarycenter * R0;
time = toc;
Yorgtransformed = bsxfun(@plus, Yorg * R0, t0);

common_plot(Xorg, Yorgtransformed, 'AfterRegistration');


