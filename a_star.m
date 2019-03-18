%distance from the point to the goal
%current dist from the start along the path
%surrounding points and their dist in a matrix
function [Mg,Mpath,path]=a_star(M,start_x,start_y,goal_x,goal_y)
%INITIALISING
[Mx_s,My_s]=size(M);
M_closed=zeros(Mx_s,My_s);
M_open=zeros(Mx_s,My_s);
Gscore=inf(Mx_s,My_s);
Sscore=inf(Mx_s,My_s);
Mscore=inf(Mx_s,My_s);
Sscore(start_x,start_y)=0;
P_x=zeros(Mx_s,My_s);
P_y=zeros(Mx_s,My_s);
m=sqrt((goal_x-start_x)^2+(goal_y-start_y)^2);
Mscore(start_x,start_y)=m;
M_open(start_x,start_y)=1;
open_list=[m start_x start_y];

%LOOP
while any(open_list)
    min_open=min(open_list); % find minimum 'm'
    [xs_open,ys_open]=size(open_list);
    if xs_open==1
        min_open=open_list;
    end
    for i=1:xs_open % find x and y coordinates corresponding to minimum 'm'
        if open_list(i,1)==min_open(1,1)
            x_c=open_list(i,2);
            y_c=open_list(i,3);
            open_list(i,:)=[]; % remove this point from the open list
            M_open(x_c,y_c)=0;
            break
        end
    end
    % create successors
    % successor 1
    if x_c~=Mx_s
    s1_x=x_c+1;
    s1_y=y_c;
    if M(s1_x,s1_y)~=1
    if s1_x==goal_x && s1_y==goal_y
        P_x(s1_x,s1_y)=x_c;
        P_y(s1_x,s1_y)=y_c;
        open_list=[];
        break
    end
    Gscore1=sqrt((goal_x-s1_x)^2+(goal_y-s1_y)^2);
    Sscore1=sqrt((x_c-s1_x)^2+(y_c-s1_y)^2)+Sscore(x_c,y_c);
    m1=Gscore1+Sscore1;
    if M_open(s1_x,s1_y)==1 && Mscore(s1_x,s1_y)<m1
    elseif M_closed(s1_x,s1_y)==1 && Mscore(s1_x,s1_y)<m1
    else
        if M_open(s1_x,s1_y)==1
            [xs_open,ys_open]=size(open_list);
            for i=1:xs_open
                if open_list(i,2)==s1_x && open_list(i,3)==s1_y
                    open_list(i,1)=m1;
                    break
                end
            end
        else
            open_list=[open_list;m1 s1_x s1_y];
            M_open(s1_x,s1_y)=1;
        end
        M_closed(s1_x,s1_y)=0;
        Gscore(s1_x,s1_y)=Gscore1;
        Sscore(s1_x,s1_y)=Sscore1;
        Mscore(s1_x,s1_y)=m1;
        P_x(s1_x,s1_y)=x_c;
        P_y(s1_x,s1_y)=y_c;
    end
    end
    end
    % successor 2
    if x_c~=Mx_s && y_c~=My_s
    s2_x=x_c+1;
    s2_y=y_c+1;
    if M(s2_x,s2_y)~=1
    if s2_x==goal_x && s2_y==goal_y
        open_list=[];
        P_x(s2_x,s2_y)=x_c;
        P_y(s2_x,s2_y)=y_c;
        break
    end
    Gscore2=sqrt((goal_x-s2_x)^2+(goal_y-s2_y)^2);
    Sscore2=sqrt((x_c-s2_x)^2+(y_c-s2_y)^2)+Sscore(x_c,y_c);
    m2=Gscore2+Sscore2;
    if M_open(s2_x,s2_y)==1 && Mscore(s2_x,s2_y)<m2
    elseif M_closed(s2_x,s2_y)==1 && Mscore(s2_x,s2_y)<m2
    else
        if M_open(s2_x,s2_y)==1
            [xs_open,ys_open]=size(open_list);
            for i=1:xs_open
                if open_list(i,2)==s2_x && open_list(i,3)==s2_y
                    open_list(i,1)=m2;
                    break
                end
            end
        else
            open_list=[open_list;m2 s2_x s2_y];
            M_open(s2_x,s2_y)=1;
        end
        M_closed(s2_x,s2_y)=0;
        Gscore(s2_x,s2_y)=Gscore2;
        Sscore(s2_x,s2_y)=Sscore2;
        Mscore(s2_x,s2_y)=m2;
        P_x(s2_x,s2_y)=x_c;
        P_y(s2_x,s2_y)=y_c;
    end
    end
    end
    % successor 3
    if y_c~=My_s
    s3_x=x_c;
    s3_y=y_c+1;
    if M(s3_x,s3_y)~=1
    if s3_x==goal_x && s3_y==goal_y
        open_list=[];
        P_x(s3_x,s3_y)=x_c;
        P_y(s3_x,s3_y)=y_c;
        break
    end
    Gscore3=sqrt((goal_x-s3_x)^2+(goal_y-s3_y)^2);
    Sscore3=sqrt((x_c-s3_x)^2+(y_c-s3_y)^2)+Sscore(x_c,y_c);
    m3=Gscore3+Sscore3;
    if M_open(s3_x,s3_y)==1 && Mscore(s3_x,s3_y)<m3
    elseif M_closed(s3_x,s3_y)==1 && Mscore(s3_x,s3_y)<m3
    else
        if M_open(s3_x,s3_y)==1
            [xs_open,ys_open]=size(open_list);
            for i=1:xs_open
                if open_list(i,2)==s3_x && open_list(i,3)==s3_y
                    open_list(i,1)=m3;
                    break
                end
            end
        else
            open_list=[open_list;m3 s3_x s3_y];
            M_open(s3_x,s3_y)=1;
        end
        M_closed(s3_x,s3_y)=0;
        Gscore(s3_x,s3_y)=Gscore3;
        Sscore(s3_x,s3_y)=Sscore3;
        Mscore(s3_x,s3_y)=m3;
        P_x(s3_x,s3_y)=x_c;
        P_y(s3_x,s3_y)=y_c;
    end
    end
    end
    % successor 4
    if x_c~=1 && y_c~=My_s
    s4_x=x_c-1;
    s4_y=y_c+1;
    if M(s4_x,s4_y)~=1
    if s4_x==goal_x && s4_y==goal_y
        open_list=[];
        P_x(s4_x,s4_y)=x_c;
        P_y(s4_x,s4_y)=y_c;
        break
    end
    Gscore4=sqrt((goal_x-s4_x)^2+(goal_y-s4_y)^2);
    Sscore4=sqrt((x_c-s4_x)^2+(y_c-s4_y)^2)+Sscore(x_c,y_c);
    m4=Gscore4+Sscore4;
    if M_open(s4_x,s4_y)==1 && Mscore(s4_x,s4_y)<m4
    elseif M_closed(s4_x,s4_y)==1 && Mscore(s4_x,s4_y)<m4
    else
        if M_open(s4_x,s4_y)==1
            [xs_open,ys_open]=size(open_list);
            for i=1:xs_open
                if open_list(i,2)==s4_x && open_list(i,3)==s4_y
                    open_list(i,1)=m4;
                    break
                end
            end
        else
            open_list=[open_list;m4 s4_x s4_y];
            M_open(s4_x,s4_y)=1;
        end
        M_closed(s4_x,s4_y)=0;
        Gscore(s4_x,s4_y)=Gscore4;
        Sscore(s4_x,s4_y)=Sscore4;
        Mscore(s4_x,s4_y)=m4;
        P_x(s4_x,s4_y)=x_c;
        P_y(s4_x,s4_y)=y_c;
    end
    end
    end
    % successor 5
    if x_c~=1
    s5_x=x_c-1;
    s5_y=y_c;
    if M(s5_x,s5_y)~=1
    if s5_x==goal_x && s5_y==goal_y
        open_list=[];
        P_x(s5_x,s5_y)=x_c;
        P_y(s5_x,s5_y)=y_c;
        break
    end
    Gscore5=sqrt((goal_x-s5_x)^2+(goal_y-s5_y)^2);
    Sscore5=sqrt((x_c-s5_x)^2+(y_c-s5_y)^2)+Sscore(x_c,y_c);
    m5=Gscore5+Sscore5;
    if M_open(s5_x,s5_y)==1 && Mscore(s5_x,s5_y)<m5
    elseif M_closed(s5_x,s5_y)==1 && Mscore(s5_x,s5_y)<m5
    else
        if M_open(s5_x,s5_y)==1
            [xs_open,ys_open]=size(open_list);
            for i=1:xs_open
                if open_list(i,2)==s5_x && open_list(i,3)==s5_y
                    open_list(i,1)=m5;
                    break
                end
            end
        else
            open_list=[open_list;m5 s5_x s5_y];
            M_open(s5_x,s5_y)=1;
        end
        M_closed(s5_x,s5_y)=0;
        Gscore(s5_x,s5_y)=Gscore5;
        Sscore(s5_x,s5_y)=Sscore5;
        Mscore(s5_x,s5_y)=m5;
        P_x(s5_x,s5_y)=x_c;
        P_y(s5_x,s5_y)=y_c;
    end
    end
    end
    % successor 6
    if x_c~=1 && y_c~=1
    s6_x=x_c-1;
    s6_y=y_c-1;
    if M(s6_x,s6_y)~=1
    if s6_x==goal_x && s6_y==goal_y
        open_list=[];
        P_x(s6_x,s6_y)=x_c;
        P_y(s6_x,s6_y)=y_c;
        break
    end
    Gscore6=sqrt((goal_x-s6_x)^2+(goal_y-s6_y)^2);
    Sscore6=sqrt((x_c-s6_x)^2+(y_c-s6_y)^2)+Sscore(x_c,y_c);
    m6=Gscore6+Sscore6;
    if M_open(s6_x,s6_y)==1 && Mscore(s6_x,s6_y)<m6
    elseif M_closed(s6_x,s6_y)==1 && Mscore(s6_x,s6_y)<m6
    else
        if M_open(s6_x,s6_y)==1
            [xs_open,ys_open]=size(open_list);
            for i=1:xs_open
                if open_list(i,2)==s6_x && open_list(i,3)==s6_y
                    open_list(i,1)=m6;
                    break
                end
            end
        else
            open_list=[open_list;m6 s6_x s6_y];
            M_open(s6_x,s6_y)=1;
        end
        M_closed(s6_x,s6_y)=0;
        Gscore(s6_x,s6_y)=Gscore6;
        Sscore(s6_x,s6_y)=Sscore6;
        Mscore(s6_x,s6_y)=m6;
        P_x(s6_x,s6_y)=x_c;
        P_y(s6_x,s6_y)=y_c;
    end
    end
    end
    % successor 7
    if y_c~=1
    s7_x=x_c;
    s7_y=y_c-1;
    if M(s7_x,s7_y)~=1
    if s7_x==goal_x && s7_y==goal_y
        open_list=[];
        P_x(s7_x,s7_y)=x_c;
        P_y(s7_x,s7_y)=y_c;
        break
    end
    Gscore7=sqrt((goal_x-s7_x)^2+(goal_y-s7_y)^2);
    Sscore7=sqrt((x_c-s7_x)^2+(y_c-s7_y)^2)+Sscore(x_c,y_c);
    m7=Gscore7+Sscore7;
    if M_open(s7_x,s7_y)==1 && Mscore(s7_x,s7_y)<m7
    elseif M_closed(s7_x,s7_y)==1 && Mscore(s7_x,s7_y)<m7
    else
        if M_open(s7_x,s7_y)==1
            [xs_open,ys_open]=size(open_list);
            for i=1:xs_open
                if open_list(i,2)==s7_x && open_list(i,3)==s7_y
                    open_list(i,1)=m7;
                    break
                end
            end
        else
            open_list=[open_list;m7 s7_x s7_y];
            M_open(s7_x,s7_y)=1;
        end
        M_closed(s7_x,s7_y)=0;
        Gscore(s7_x,s7_y)=Gscore7;
        Sscore(s7_x,s7_y)=Sscore7;
        Mscore(s7_x,s7_y)=m7;
        P_x(s7_x,s7_y)=x_c;
        P_y(s7_x,s7_y)=y_c;
    end
    end
    end
    % successor 8
    if x_c~=Mx_s && y_c~=1
    s8_x=x_c+1;
    s8_y=y_c-1;
    if M(s8_x,s8_y)~=1
    if s8_x==goal_x && s8_y==goal_y
        open_list=[];
        P_x(s8_x,s8_y)=x_c;
        P_y(s8_x,s8_y)=y_c;
        break
    end
    Gscore8=sqrt((goal_x-s8_x)^2+(goal_y-s8_y)^2);
    Sscore8=sqrt((x_c-s8_x)^2+(y_c-s8_y)^2)+Sscore(x_c,y_c);
    m8=Gscore8+Sscore8;
    if M_open(s8_x,s8_y)==1 && Mscore(s8_x,s8_y)<m8
    elseif M_closed(s8_x,s8_y)==1 && Mscore(s8_x,s8_y)<m8
    else
        if M_open(s8_x,s8_y)==1
            [xs_open,ys_open]=size(open_list);
            for i=1:xs_open
                if open_list(i,2)==s8_x && open_list(i,3)==s8_y
                    open_list(i,1)=m8;
                    break
                end
            end
        else
            open_list=[open_list;m8 s8_x s8_y];
            M_open(s8_x,s8_y)=1;
        end
        M_closed(s8_x,s8_y)=0;
        Gscore(s8_x,s8_y)=Gscore8;
        Sscore(s8_x,s8_y)=Sscore8;
        Mscore(s8_x,s8_y)=m8;
        P_x(s8_x,s8_y)=x_c;
        P_y(s8_x,s8_y)=y_c;
    end
    end
    end
    M_closed(x_c,y_c)=1;
end
%RECONSTRUCTION
x_b=goal_x;
y_b=goal_y;
Mg=M;
Mpath=zeros(Mx_s,My_s);
path=[goal_x,goal_y];
while true
    xparent=P_x(x_b,y_b);
    yparent=P_y(x_b,y_b);
    Mg(xparent,yparent)=1;
    Mpath(xparent,yparent)=1;
    path=[xparent,yparent;path];
    x_b=xparent;
    y_b=yparent;
    if xparent==start_x && yparent==start_y
        break
    end
end
end