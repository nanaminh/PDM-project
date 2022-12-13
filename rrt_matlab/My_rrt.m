%***************************************
%Author: Danning Zhao
%Date: 12/12/2022
%***************************************
%% 
clc
clear all; close all;
x_I=1; y_I=1;          
x_G=700; y_G=700;      
Thr=50;                
Delta= 30;              
%%
T.v(1).x = x_I;        
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;    
T.v(1).yPrev = y_I;
T.v(1).dist=0;          
T.v(1).indPrev = 0;     
%% 
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,2);%
yL=size(Imp,1);%
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');%
count=1;
bFind = false;

get_dist = @(x1,y1,x2,y2) sqrt((x1-x2)^2 + (y1-y2)^2);

for iter = 1:3000
   
    %Step 1: x_rand
     x_rand = [rand()*xL,rand()*yL];
    %plot(x_rand(1), x_rand(2), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
    
    
    %Step 2: x_near 
    [~,N]=size(T.v);
    x_distance=10000;%set init distance
    index_temp=1;
    %get the index of nearest node
    for index=1:N
        x_new_distance=get_dist(T.v(index).x,T.v(index).y,x_rand(1),x_rand(2));
        if x_new_distance<x_distance
            x_distance=x_new_distance;
            index_temp=index;%update the index
        end
    end




    %Step 3: x_new
    x_near(1)=T.v(index_temp).x;
    x_near(2)=T.v(index_temp).y;
%     cosine=(x_rand(1)-x_near(1))/x_distance;
%     sinus=(x_rand(2)-x_near(2))/x_distance;
%     x_new(1)=x_near(1)+Delta*cosine;
%     x_new(2)=x_near(2)+Delta*sinus;
    x_new=[];
    near_to_rand=[x_rand(1)-x_near(1),x_rand(2)-x_near(2)];
    normalized=near_to_rand/norm(near_to_rand)*Delta;
    x_new=x_near+normalized;
    %check collision-free
    feasible=collisionChecking(x_near,x_new,Imp);
    %fprintf(feasible);
    if ~feasible
        continue;
    end
    %Step 4: push x_new into the tree T 
   
    T.v(N+1).x = x_new(1);         
    T.v(N+1).y = x_new(2); 
    T.v(N+1).xPrev = x_near(1);     
    T.v(N+1).yPrev = x_near(2);
    T.v(N+1).dist=get_dist(x_near(1),x_near(2),x_new(1),x_new(2));          
    T.v(N+1).indPrev = index_temp;     %
    %plot(x_new(1), x_new(2), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');



    if get_dist(x_new(1),x_new(2),x_G,y_G)<Thr
        bFind=true;
        break;
    end
    
    plot([x_near(1),x_new(1)],[x_near(2),x_new(2)],"r");
     hold on
   
     pause(0.05); 


end
%% path found, back tracing
if bFind
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % 
    j=0;
    while 1
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % 
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % 
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end
