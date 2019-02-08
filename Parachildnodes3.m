%% 找到子代状态机
function [S,Act]=Parachildnodes3(s,Readytime)
%s为状态矩阵，8~9位表示缓存的工件要放的位置，10~11位表示act
%% 找到子代状态机
M=7;     % 加工工序个数
RM=11;
% s=[0     0     0     2     2     1     0     4     0];
% Readytime=zeros(1,RM+4);
% Readytime(9:10)=[100 100];
% Readytime(11:12)=[120 120];
% Readytime(13)=140;
A=zeros(M,M);
for i=1:M-1
    A(i,i)=-1;
    A(i,i+1)=1;
end
A(M,M)=-1;
tr1=1;% 取片时间
tr2=1;% 放片时间
tr3=1;% 旋转时间
% 工序时间
MaxS=[5,2,1,2,2,1,2];%表示每个工序最多容纳工件个数
Act=zeros(1,2);%act=i表示从工作腔i-1到工作腔i，即从state的i到i+1
Act2=zeros(1,2);
z=zeros(1,M+2); %zi从第i-1个腔拿起工件放到第i个腔,最后2列为buffer位，1表示双臂已经拿了一个工件，0表示空
F1=[find(s(1:2)) find(s(6:M))+5]; % 寻找所有不为0的位置
F2=find(s(3:6))+2;
Len1=length(F1);
Len2=length(F2);
flag1=0; % flag=0无buffer,1表示buffer1可以放，2表示有buffer但不能放
flag2=0; % flag=0无buffer,1表示buffer2可以放，2表示有buffer但不能放

if(Len1>0)
    for i=1:Len1
         if(F1(i)==M)
            z(M)=1;
        else
            if(s(F1(i)+1)<MaxS(F1(i)+1))
                z(F1(i))=1;
            end
         end
    end
end

if(Len2>0)
    if(s(M+1)>0)
        if(s(s(M+1))<MaxS(s(M+1)))
            %放buffer1工件
            flag1=1;
        else
            flag1=2;
        end
    end
    if(s(M+2)>0)
        if(s(s(M+2))<MaxS(s(M+2)))
            %放buffer2工件
            flag2=1;
        else
            flag2=2;
        end
    end
end

z1=[find(z(1:2)) find(z(6:M))+5];
Z=zeros(1,M); % 动作向量    
k2=2; % 子代编号
S2(1,:)=s(1,:); % S2为工作台2的所有可能状态
k=2; % 子代编号
S(1,1:M+2+2)=s(1,1:M+2+2); % S为所有子代
if(flag1==1)
    %放buffer1
    S2(k2,:)=s;
    S2(k2,s(M+1))=s(s(M+1))+1;
    S2(k2,M+1)=0;
    Act2(k2,2)=10+s(M+1);
    k2=k2+1;
end

if(flag2==1)
    %放buffer2
    S2(k2,:)=s;
    S2(k2,s(M+2))=s(s(M+2))+1;
    S2(k2,M+2)=0;
    Act2(k2,2)=10+s(M+2);
    k2=k2+1;
end

minP=zeros(1,M+2);
minP(3)=Readytime(8);
minP(4)=min(Readytime(9),Readytime(10));
minP(5)=min(Readytime(11),Readytime(12));
minP(6)=Readytime(13);
maxP=zeros(1,M+2);
maxP(3)=Readytime(8);
maxP(4)=max(Readytime(9),Readytime(10));
maxP(5)=max(Readytime(11),Readytime(12));
maxP(6)=Readytime(13);
buffer=[];
bufferi=1;
if(flag1~=1 && flag2~=1)
    %如果没有buffer可以放
    if(Len2>1)
        for F2i=Len2:-1:2
            if(F2(F2i)-F2(F2i-1)==1)
                %如果为2个相邻为1的 
                if(s(F2(F2i-1))==MaxS(F2(F2i-1)) && s(F2(F2i))==MaxS(F2(F2i)) && minP(F2(F2i))>minP(F2(F2i-1)))
                    buffer(bufferi)=F2(F2i-1);
                    bufferi=bufferi+1;
                elseif(s(F2(F2i-1))<MaxS(F2(F2i-1)) && s(F2(F2i))==MaxS(F2(F2i)) && minP(F2(F2i))>maxP(F2(F2i-1)))
                    buffer(bufferi)=F2(F2i-1);
                    bufferi=bufferi+1;
                end
            end
        end
%         if(~isempty(buffer) && flag1==0 && flag2==0)
        if(~isempty(buffer))
            %机械双臂有空闲，要抓取工件作为buffer
            for bufferi2=1:bufferi-1
                S2(k2,:)=s;
                S2(k2,buffer(bufferi2))=S2(k2,buffer(bufferi2))-1;
                if(flag1==0&&flag2==0)
                    S2(k2,M+1)=buffer(bufferi2)+1;
                    Act2(k2,2)=20+buffer(bufferi2);
                    k2=k2+1;
                elseif(flag1~=0)
                    if(buffer(bufferi2)==s(M+1)||buffer(bufferi2)==5)
                        S2(k2,M+2)=buffer(bufferi2)+1;
                        Act2(k2,2)=20+buffer(bufferi2);
                        k2=k2+1;
                    end
                else
                    if(buffer(bufferi2)==s(M+2)||buffer(bufferi2)==5)
                        S2(k2,M+1)=buffer(bufferi2)+1;                        
                        Act2(k2,2)=20+buffer(bufferi2);
                        k2=k2+1;
                    end
                end
            end
        end      
    end
    if(Len2>0 && F2(Len2)==6)
        Len2=Len2-1;
    end
    if(Len2>0)
        %作为单臂工作
        for F2i=1:Len2            
            if(s(F2(F2i))>0 && s(F2(F2i)+1)<MaxS(F2(F2i)+1))
                Z(F2(F2i))=1;
                S2(k2,:)=s;
                S2(k2,1:M)=s(1,1:M)+Z*A;
                Act2(k2,2)=F2(F2i);
                k2=k2+1;
                Z=zeros(1,M);
            end
        end
    end
end


if(~isempty(z1))
    %只有机械臂1操作
    for i=1:length(z1)
        Z(z1(i))=1;
        S(k,1:M)=s(1,1:M)+Z*A;
        S(k,M+1:M+2)=s(1,M+1:M+2);
        Z=zeros(1,M);
        Act(k,1)=z1(i);
        Act(k,2)=0;
        k=k+1;
    end
end
if(k2>2)
    %只有机械臂2操作
    for Si=2:k2-1
        S(k,:)=S2(Si,:);
        Act(k,1)=0;
        Act(k,2)=Act2(Si,2);
        k=k+1;
    end
% else
%     %两臂同时操作
%     for SB=1:length(z1)
%         Z(z1(SB))=1;    
%         for Si=2:k2-1
%             S(k,1:M)=s(1,1:M)+Z*A;
%             if z1(SB)~=2 && Act2(Si,2)~=5
%                 S(k,3:5)=S2(Si,3:5);
%             else
%                 if z1(SB)==2 && Act2(Si,2)~=5
%                     S(k,4:5)=S2(Si,4:5);
%                 end
%                 if Act2(Si,2)==5 && z1(SB)~=2
%                     S(k,3:6)=S2(Si,3:6);
%                 end
%                 if z1(SB)==2 && Act2(Si,2)==5
%                     S(k,4:6)=S2(Si,4:6);
%                 end
%             end
%             S(k,M+1:M+2)=S2(Si,M+1:M+2);
%             Act(k,1)=z1(SB);
%             Act(k,2)=Act2(Si,2);
%             k=k+1;
%         end
%         Z=zeros(1,M);
%     end
%         
end
S(1,:)=[];
Act(1,:)=[];
S(:,M+3:M+4)=Act(:,1:2);
end