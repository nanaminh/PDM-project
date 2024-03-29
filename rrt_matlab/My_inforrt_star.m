%***************************************
%Author: Diane
%Date: 7/12/2022
%***************************************
%
%% 1.initialization the project
clc
clear all; close all;
x_I=1; y_I=1;           % set initialization point
x_G=300; y_G=700;       %set goal
Thr=50;                 % s
Delta= 30;              % set step size
%% 2.initialization of the tree
T.v(1).x = x_I;         % T:tree, v: list of nodes
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     % the
T.v(1).yPrev = y_I;
T.v(1).dist=0;          % the distance from root to the current node
T.v(1).indPrev = 0;     %
%% 3. reading the map
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,2);%x length of map
yL=size(Imp,1);%y length of map
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');%
count=1;
bFind = false;

get_dist = @(x1,y1,x2,y2) sqrt((x1-x2)^2 + (y1-y2)^2);

radius=120;%set the search radius 
first_arrive=true;
xgoal_index=0;%store the index of node_goal
distance_toroot=1000000;
for iter = 1:3000
   
    %Step 1: find x_rand in the map
    %x_rand(1),x_rand(2)）
     x_rand=informed_sample([x_I,y_I],[x_G,y_G],distance_toroot,xL,yL);
    %plot(x_rand(1), x_rand(2), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
    
    
    %Step 2: find the nearest node x_near 
    %
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

    %Step 3: get x_new node
    %Delta 
    x_near(1)=T.v(index_temp).x;
    x_near(2)=T.v(index_temp).y;

    x_new=[];
    near_to_rand=[x_rand(1)-x_near(1),x_rand(2)-x_near(2)];
    normalized=near_to_rand/norm(near_to_rand)*Delta;
    x_new=x_near+normalized;
    %Step4.1:check node is whether collision-free
    feasible=collisionChecking(x_near,x_new,Imp);
      if ~feasible
        continue;
      end
    %fprintf(feasible);
    %Step 4.2: search possible parents nodes in a certain radius
    %
    x_near_list=[];%store possible parents in the radius
    near_count=0;%record the number of possible parents in certain radius
    near_index=index_temp;%the default index is the index of the nereast node index
    x_dist=norm(x_new-x_near)+T.v(index_temp).dist;
    %record the distance from x_near to root
    for index=1:N
        if index==index_temp
            continue;
        end
        x_near_temp=[];
        x_near_temp(1)=T.v(index).x;
        x_near_temp(2)=T.v(index).y;
        x_near_dist=norm(x_near_temp-x_new);%
        dist=norm(x_new-x_near_temp)+T.v(index).dist;
        if x_near_dist<radius
            if collisionChecking(x_near_temp,x_new,Imp)%if no collision, then the new x_near is possible
                x_near_list(near_count+1)=index;
                near_count=near_count+1;
                if dist<x_dist
                    x_dist=dist;
                    near_index=index;%store the nearest index
                end
            end
        end
    end

    
    %Step4.3:get x_near, then set it as parent of x_new, then push the x_new
    %into the tree
  
    x_near(1)=T.v(near_index).x;
    x_near(2)=T.v(near_index).y;
    T.v(N+1).x = x_new(1);         
    T.v(N+1).y = x_new(2); 
    T.v(N+1).xPrev = x_near(1);     
    T.v(N+1).yPrev = x_near(2);
    T.v(N+1).dist=T.v(near_index).dist+norm(x_near-x_new); 
    T.v(N+1).indPrev = near_index;     %


%Step5: rewiring
    [~,M]=size(x_near_list);
    x_near_list(M+1)=index_temp;

    for index =1:M+1
        x_l(1)=T.v(x_near_list(index)).x;
        x_l(2)=T.v(x_near_list(index)).y;
        xl_prev(1)=T.v(x_near_list(index)).xPrev;%
        xl_prev(2)=T.v(x_near_list(index)).yPrev;
        near_dist=T.v(x_near_list(index)).dist;
        new_dist=T.v(N+1).dist+norm(x_l-x_new);%
        if near_dist>new_dist
            %choose the parent of near node to new node
            T.v(x_near_list(index)).xPrev = x_new(1);    
            T.v(x_near_list(index)).yPrev = x_new(2);
            T.v(x_near_list(index)).dist=new_dist;%
            T.v(x_near_list(index)).indPrev = N+1;     %
            plot([x_l(1),xl_prev(1)],[x_l(2),xl_prev(2)],"-w");
            hold on;
            plot([x_l(1),x_new(1)],[x_l(2),x_new(2)],"-g");
        end
    end

    plot([x_near(1),x_new(1)],[x_near(2),x_new(2)],"-r")
    hold on;
    plot(x_new(1),x_new(2),"*r");
    hold on;
    pause(0.05);
    goal(1)=x_G;
    goal(2)=y_G;
    if norm(x_new-goal)<Thr
        bFind=true;
    end
    %% After we find the path, we search back from the goal node
    if bFind
        if first_arrive
            %push the goal node into the tree
            T.v(N+2).x = x_G;        
            T.v(N+2).y = y_G; 
            T.v(N+2).xPrev = x_new(1);     % 
            T.v(N+2).yPrev = x_new(2);
            T.v(N+2).dist=T.v(N+1).dist+norm(x_new-goal); 
            T.v(N+2).indPrev = N+1;   
            xgoal_index=N+2;%
            %%%

            path.pos(1).x = x_G; path.pos(1).y = y_G;%
            pathIndex=N+1;
            j=2;
            while 1
                path.pos(j).x = T.v(pathIndex).x;
                path.pos(j).y = T.v(pathIndex).y;
                pathIndex = T.v(pathIndex).indPrev;
                if pathIndex == 1
                    break%if we arrive at the initial node
                end
                j=j+1;
            end  
            
            %
            path.pos(end+1).x = x_I; path.pos(end).y = y_I; % push the initial node
            first_arrive=false;%we already arrived at the goal, then we will updating our path
%visualisation
            for j = 2:length(path.pos)
                plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b','Linewidth', 2);
                hold on;
                pause(0.05);
            end
        distance_toroot=T.v(N+2).dist;%distance from goal to root
        disp("path found!");
        disp(T.v(N+2).dist);

        else%first_arrive=false, we are updating the path
            new_distance_toroot=T.v(xgoal_index).dist;%if the new path length is less than the 
            % old one ,we update the path
            if new_distance_toroot<distance_toroot
                distance_toroot=new_distance_toroot;
                path.pos=[];%
                %back tracing
                path.pos(1).x = x_G; path.pos(1).y = y_G;%
                pathIndex=T.v(xgoal_index).indPrev;%not N+1, because might updated
                j=2;
                 while 1
                    path.pos(j).x = T.v(pathIndex).x;
                    path.pos(j).y = T.v(pathIndex).y;
                    pathIndex = T.v(pathIndex).indPrev;
                    if pathIndex == 1
                        break
                    end
                    j=j+1;
                 end  
                 %visualization
                path.pos(end+1).x = x_I; path.pos(end).y = y_I; 
                first_arrive=false;
                for j = 2:length(path.pos)
                    plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b','Linewidth', 2);
                    hold on;

                end
                disp("new path found!");
                disp(T.v(xgoal_index).dist);
            end
        end
    end
end

if ~bFind
    disp('Error, no path found!');
end