%% 串行
clear all;
tic
%% 初始参数
RM=11;   % 腔室个数
M=7;     % 加工工序个数
s=zeros(1,M+2+2);   % 状态机,每个工作腔中工件个数,8~9位为buffer,10~11位表示act
MaxS=[5,2,1,2,2,1,2];%表示每个工序最多容纳工件个数
s(1)=5;%s(3)=1;s(5)=1;s(7)=1;s(11)=1;
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
PMTime=[0,139,0,131,130,0,120];
% PMTime=[0,70,0,50,60,0,50];
Readytime=zeros(1,RM+4);%RT[1]为父代序号，2为子代序号，3为父代层数，4，5是Robot时间，6-15是10道工序时间
Readytime(2)=1;
Makespan=10000;
% z=zeros(1,M); %zi从第i-1个腔拿起工件放到第i个腔
%% 并行
Parent={s};
Generation=1;
Readym=2;
Allnodes=zeros(1,2+M+2+2);%所有节点，用于比较查重，第一列父节点所在层数，第二列为自己在自己层数内序号，之后为状态机
Allnodes(1)=1;Allnodes(2)=1;Allnodes(3)=5;
allnodesm=2;
while(1)
    P=Parent{Generation};
    [PH,PL]=size(P);
    TempReadyTime=zeros(1,RM+2+M+2+2);% 存同一层父代所有的子代的readytime,后11位为状态机
    TRTm=1;%Tempreadytime编号
    for i=1:PH
        Temptime=zeros(1,RM+1);%存取子代readytime.1,2为robot时间
        k=find(Readytime(:,2)==i & Readytime(:,3)==(Generation-1));
        PReadytime=Readytime(k,:);
        [child,act]=Parachildnodes3(P(i,:),PReadytime);
        [CH,CL]=size(child);
         minP=zeros(2,M+2); 
         maxP=zeros(2,M+2);
         if Readytime(k,6)<=Readytime(k,7)
             minP(1,2)=Readytime(k,6);
             minP(2,2)=3;
             maxP(1,2)=Readytime(k,7);
             maxP(2,2)=4;
         else
             minP(1,2)=Readytime(k,7);
             minP(2,2)=4;
             maxP(1,2)=Readytime(k,6);
             maxP(2,2)=3;
         end
         minP(:,3)=[Readytime(k,8); 5];
         maxP(:,3)=[Readytime(k,8); 5];
         if Readytime(k,9)<=Readytime(k,10)
             minP(1,4)=Readytime(k,9);
             minP(2,4)=6;
             maxP(1,4)=Readytime(k,10);
             maxP(2,4)=7;
         else
             minP(1,4)=Readytime(k,10);
             minP(2,4)=7;
             maxP(1,4)=Readytime(k,9);
             maxP(2,4)=6;
         end
         if Readytime(k,11)<=Readytime(k,12)
             minP(1,5)=Readytime(k,11);
             minP(2,5)=8;
             maxP(1,5)=Readytime(k,12);
             maxP(2,5)=9;
         else
             minP(1,5)=Readytime(k,12);
             minP(2,5)=9;
             maxP(1,5)=Readytime(k,11);
             maxP(2,5)=8;
         end
         minP(:,6)=[Readytime(k,13); 10];
         maxP(:,6)=[Readytime(k,13); 10];
         if Readytime(k,14)<=Readytime(k,15)
             minP(1,7)=Readytime(k,14);
             minP(2,7)=11;
             maxP(1,7)=Readytime(k,15);
             maxP(2,7)=12;
         else
             minP(1,7)=Readytime(k,15);
             minP(2,7)=12;
             maxP(1,7)=Readytime(k,14);
             maxP(2,7)=11;
         end
        for j=1:CH        
            Temptime(j,:)=Readytime(k,4:RM+4);             
               if(act(j,2)~=0)
                if(act(j,2)>=14 && act(j,2)<=16)
                    %双臂放buffer
                    Temptime(j,2)=max(Readytime(k,5),minP(1,act(j,2)-10))+2;
                    Temptime(j,minP(2,act(j,2)-10))=max(Readytime(k,5),Temptime(j,minP(2,act(j,2)-10)))+PMTime(act(j,2)-10)+1;
                elseif(act(j,2)>=23 && act(j,2)<=25)   
                    %取buffer
                    if(P(i,act(j,2)-20)==MaxS(act(j,2)-20))
                        Temptime(j,2)=max(Readytime(k,5),minP(1,act(j,2)-20))+2;
                        Temptime(j,minP(2,act(j,2)-20))=max(Readytime(k,5),Temptime(j,minP(2,act(j,2)-20)))+1;
                    else
                        Temptime(j,2)=max(Readytime(k,5),maxP(1,act(j,2)-20))+2;
                        Temptime(j,maxP(2,act(j,2)-20))=max(Readytime(k,5),Temptime(j,maxP(2,act(j,2)-20)))+1;
                    end
                else %作为单臂使用
                    if(P(i,act(j,2))==MaxS(act(j,2)))
                        Temptime(j,2)=max(Readytime(k,5),max(minP(1,act(j,2)),minP(1,act(j,2)+1)))+4;
                        Temptime(j,minP(2,act(j,2)+1))=max(Readytime(k,5),max(minP(1,act(j,2)),minP(1,act(j,2)+1)))+PMTime(act(j,2)+1)+3;
                    else
                        Temptime(j,2)=max(Readytime(k,5),max(maxP(1,act(j,2)),minP(1,act(j,2)+1)))+4;
                        Temptime(j,minP(2,act(j,2)+1))=max(Readytime(k,5),max(maxP(1,act(j,2)),minP(1,act(j,2)+1)))+PMTime(act(j,2)+1)+3;
                    end
                end
              end
              if(act(j,1)~=0)
                if(act(j,1)==1)
                    Temptime(j,1)=max(Readytime(k,4),minP(1,act(j,1)+1))+4;
                    Temptime(j,minP(2,act(j,1)+1))=max(Readytime(k,4),minP(1,act(j,1)+1))+PMTime(2)+3;
                elseif(act(j,1)==M)
                    if(P(i,act(j,1))==MaxS(act(j,1)))
                        Temptime(j,1)=max(Readytime(k,4),minP(1,M))+4;
                        Temptime(j,minP(2,M))=max(Readytime(k,4),minP(1,M))+1;
                    else
                        Temptime(j,1)=max(Readytime(k,4),maxP(1,M))+4;
                        Temptime(j,maxP(2,M))=max(Readytime(k,4),maxP(1,M))+1;
                    end
                else
                    if(P(i,act(j,1))==MaxS(act(j,1)))
                        Temptime(j,1)=max(Readytime(k,4),max(minP(1,act(j,1)),minP(1,act(j,1)+1)))+4;
                        Temptime(j,minP(2,act(j,1)+1))=max(Readytime(k,4),max(minP(1,act(j,1)),minP(1,act(j,1)+1)))+PMTime(act(j,1)+1)+3;
                    else
                        Temptime(j,1)=max(Readytime(k,4),max(maxP(1,act(j,1)),maxP(1,act(j,1)+1)))+4;
                        Temptime(j,minP(2,act(j,1)+1))=max(Readytime(k,4),max(maxP(1,act(j,1)),minP(1,act(j,1)+1)))+PMTime(act(j,1)+1)+3;
                    end
                end
              end
              for m=3:4
                  Temptime(j,m)=max(Temptime(j,1),Temptime(j,m));
              end
              for m=11:12
                  Temptime(j,m)=max(Temptime(j,1),Temptime(j,m));
              end
              for m=6:9
                  Temptime(j,m)=max(Temptime(j,2),Temptime(j,m));
              end
              Temptime(j,5)=max(Temptime(j,5),min(Temptime(j,1),Temptime(j,2)));
              Temptime(j,10)=max(Temptime(j,10),min(Temptime(j,1),Temptime(j,2)));
              TempReadyTime(TRTm,2:RM+2)=Temptime(j,:);
              TempReadyTime(TRTm,1)=i; %TempReadyTime的第一列为当前子代们的父代的编号
              TempReadyTime(TRTm,RM+3:end)=child(j,:);
              TRTm=TRTm+1;
        end
    end
    %%  找到Tempreadytime中的相同子代，删去被支配的路径
        Tag=ones(1,TRTm-1);
        [~,diffm,diffn]=unique(TempReadyTime(:,RM+3:RM+11),'rows'); %diffm为含有不同元素的行号，diffn为编号
        for tempmini=1:size(TempReadyTime,1)
            minTempReadyTime(tempmini,1:3)=TempReadyTime(tempmini,1:3);
            minTempReadyTime(tempmini,4)=min(TempReadyTime(tempmini,4),TempReadyTime(tempmini,5));
            minTempReadyTime(tempmini,5)=minTempReadyTime(tempmini,4);
            minTempReadyTime(tempmini,6)=TempReadyTime(tempmini,6);
            minTempReadyTime(tempmini,7)=min(TempReadyTime(tempmini,7),TempReadyTime(tempmini,8));
            minTempReadyTime(tempmini,8)=minTempReadyTime(tempmini,7);
            minTempReadyTime(tempmini,9)=min(TempReadyTime(tempmini,9),TempReadyTime(tempmini,10));
            minTempReadyTime(tempmini,10)=minTempReadyTime(tempmini,9);
            minTempReadyTime(tempmini,11)=TempReadyTime(tempmini,11);
            minTempReadyTime(tempmini,12)=min(TempReadyTime(tempmini,12),TempReadyTime(tempmini,13));
            minTempReadyTime(tempmini,13)=minTempReadyTime(tempmini,12);
            maxTempReadyTime(tempmini,1:3)=TempReadyTime(tempmini,1:3);
            maxTempReadyTime(tempmini,4)=max(TempReadyTime(tempmini,4),TempReadyTime(tempmini,5));
            maxTempReadyTime(tempmini,5)=maxTempReadyTime(tempmini,4);
            maxTempReadyTime(tempmini,6)=TempReadyTime(tempmini,6);
            maxTempReadyTime(tempmini,7)=max(TempReadyTime(tempmini,7),TempReadyTime(tempmini,8));
            maxTempReadyTime(tempmini,8)=maxTempReadyTime(tempmini,7);
            maxTempReadyTime(tempmini,9)=max(TempReadyTime(tempmini,9),TempReadyTime(tempmini,10));
            maxTempReadyTime(tempmini,10)=maxTempReadyTime(tempmini,9);
            maxTempReadyTime(tempmini,11)=TempReadyTime(tempmini,11);
            maxTempReadyTime(tempmini,12)=max(TempReadyTime(tempmini,12),TempReadyTime(tempmini,13));
            maxTempReadyTime(tempmini,13)=maxTempReadyTime(tempmini,12);
        end
        if(size(diffm,1)~=TRTm-1)
            %有相同子代
            for i=1:max(diffn)
                sameh=find(diffn==i);
                if(size(sameh,1)>1)    
                %    if(TempReadyTime(sameh(1),14:25)==0)
                   % else
                        for j=1:size(sameh,1)-1
                            for n=j+1:size(sameh,1)
                                if(all(minTempReadyTime(sameh(j),2:RM+2)>=minTempReadyTime(sameh(n),2:RM+2))&&all(maxTempReadyTime(sameh(j),2:RM+2)>=maxTempReadyTime(sameh(n),2:RM+2)))
                                    Tag(sameh(j))=0;
                                elseif(all(maxTempReadyTime(sameh(j),2:RM+2)<=maxTempReadyTime(sameh(n),2:RM+2))&&all(minTempReadyTime(sameh(j),2:RM+2)<=minTempReadyTime(sameh(n),2:RM+2)))
                                    Tag(sameh(n))=0;
                                end
                            end
                        end
                   % end
                end
            end
        end
        remain=find(Tag==1);    
    
    %%  剩余子代时间存到Readytime中，状态存到Parent中
        Tempchild=zeros(1,M+2+2);
        m=1;
        for j=1:length(remain)
            Readytime(Readym,1)=TempReadyTime(remain(j),1);
            Readytime(Readym,2)=j;
            Readytime(Readym,3)=Generation;
            Readytime(Readym,4:RM+4)=TempReadyTime(remain(j),2:RM+2);
            Tempchild(m,1:end)=TempReadyTime(remain(j),RM+3:end);
            Readym=Readym+1;
            Allnodes(allnodesm,1)=Generation;
            Allnodes(allnodesm,2)=j;
            Allnodes(allnodesm,3:M+4+2)=TempReadyTime(remain(j),RM+3:end);
            allnodesm=allnodesm+1;
            m=m+1;
        end
        Generation=Generation+1;
        Parent(Generation)={Tempchild};
        [sizeTempchild1,sizeTempchild2]=size(Tempchild);
        if(sizeTempchild1==1)
            if(Tempchild==0) 
                break;
            end
        end
end
% terminate=find(all(Allnodes(:,3:end)==0));
what=1;
for i=1:size(Allnodes,1)
    if(Allnodes(i,3:M+3)==0)
        row(what)=i;
        what=what+1;
    end
end
% tempgen=Parent{Generation};
% route(1,:)=tempgen(Readytime(timerow,2),:);
route(1,:)=zeros(1,M+2+2);
for rowmin=1:what-1
    timerowtemp=find(Readytime(:,2)==Allnodes(row(rowmin),2) & Readytime(:,3)==Allnodes(row(rowmin),1));
    Makespantemp=max(Readytime(timerowtemp,4),Readytime(timerowtemp,5));
    if(Makespantemp<Makespan)
        Makespan=Makespantemp;
        timerow=timerowtemp;
        gen=Readytime(timerowtemp,3);
    end
end
time(1)=Makespan;
routei=2;
timei=2;
parentnum=Readytime(timerow,1);
PPP(1,:)=Readytime(timerow,:);
while(1)
    gen=gen-1;
    timerow=find(Readytime(:,2)==parentnum & Readytime(:,3)==gen);
    parentnum=Readytime(timerow,1);
    tempgen=Parent{gen+1};
    route(routei,:)= tempgen(Readytime(timerow,2),:);
    time(timei)=max(Readytime(timerow,4),Readytime(timerow,5));
    PPP(timei,:)=Readytime(timerow,:);
    timei=timei+1;
    routei=routei+1;
    if(gen==0)
        break;
    end
end
RouteFinal=route(end:-1:1,:);
PT=PPP(end:-1:1,:);
%% 输出动作序列

ActTemp=RouteFinal(:,end-1:end);
ActFinal(:,1)=ActTemp(:,1);
ActFinal(:,2)=ActTemp(:,1);
ActFinal(:,3)=ActTemp(:,2);
ActFinal(:,4)=ActTemp(:,2);
ActFinal(ActFinal==7)=10;
ActFinal(ActFinal==6)=9;
ActFinal(ActFinal==5)=7;
ActFinal(ActFinal==4)=5;
ActFinal(ActFinal==3)=4;
for i=2:size(ActTemp,1)
     minP=zeros(2,M+2); 
     maxP=zeros(2,M+2);
     minP(2,1)=1;
     maxP(2,1)=1;
     if PT(i-1,6)<=PT(i-1,7)
         minP(1,2)=PT(i-1,6);
         minP(2,2)=2;
         maxP(1,2)=PT(i-1,7);
         maxP(2,2)=3;
     else
         minP(1,2)=PT(i-1,7);
         minP(2,2)=3;
         maxP(1,2)=PT(i-1,6);
         maxP(2,2)=2;
     end
     minP(:,3)=[PT(i-1,8); 4];
     maxP(:,3)=[PT(i-1,8); 4];
     if PT(i-1,9)<=PT(i-1,10)
         minP(1,4)=PT(i-1,9);
         minP(2,4)=5;
         maxP(1,4)=PT(i-1,10);
         maxP(2,4)=6;
     else
         minP(1,4)=PT(i-1,10);
         minP(2,4)=6;
         maxP(1,4)=PT(i-1,9);
         maxP(2,4)=5;
     end
     if PT(i-1,11)<=PT(i-1,12)
         minP(1,5)=PT(i-1,11);
         minP(2,5)=7;
         maxP(1,5)=PT(i-1,12);
         maxP(2,5)=8;
     else
         minP(1,5)=PT(i-1,12);
         minP(2,5)=8;
         maxP(1,5)=PT(i-1,11);
         maxP(2,5)=7;
     end
     minP(:,6)=[PT(i-1,13); 9];
     maxP(:,6)=[PT(i-1,13); 9];
     if PT(i-1,14)<=PT(i-1,15)
         minP(1,7)=PT(i-1,14);
         minP(2,7)=10;
         maxP(1,7)=PT(i-1,15);
         maxP(2,7)=11;
     else
         minP(1,7)=PT(i-1,15);
         minP(2,7)=11;
         maxP(1,7)=PT(i-1,14);
         maxP(2,7)=10;
     end
     
     if(ActTemp(i,1)~=0)
       if(RouteFinal(i-1,ActTemp(i,1))<MaxS(ActTemp(i,1)))
           ActFinal(i,1)=maxP(2,ActTemp(i,1));
       else
            ActFinal(i,1)=minP(2,ActTemp(i,1));
       end
       ActFinal(i,2)=minP(2,ActTemp(i,1)+1);
     end
     if(ActTemp(i,2)~=0 && ActTemp(i,2)<10)
       if(RouteFinal(i-1,ActTemp(i,2))<MaxS(ActTemp(i,2)))
           ActFinal(i,3)=maxP(2,ActTemp(i,2));
       else
            ActFinal(i,3)=minP(2,ActTemp(i,2));
       end
       ActFinal(i,4)=minP(2,ActTemp(i,2)+1);
     end
     if(ActTemp(i,2)>=14&&ActTemp(i,2)<=16)
         %放buffer
         ActFinal(i,3)=100;%100表示在机械臂上
         ActFinal(i,4)=minP(2,ActTemp(i,2)-10);
     end
     if(ActTemp(i,2)>=23&&ActTemp(i,2)<=25)
         %取buffer
         ActFinal(i,4)=100;%100表示在机械臂上
         if(RouteFinal(i-1,ActTemp(i,2)-20)<MaxS(ActTemp(i,2)-20))
           ActFinal(i,3)=maxP(2,ActTemp(i,2)-20);
         else
           ActFinal(i,3)=minP(2,ActTemp(i,2)-20);
         end
     end
     ActFinal(36,1)=10;
end
ActFinal(end,1)=11;
ActFinal(1,:)=[];
for i=2:size(PT,1)
    if RouteFinal(i,10)~=0
        if(RouteFinal(i,10)==1)
            timerobot(i-1)=max(PT(i-1,4),min(PT(i-1,6),PT(i-1,7)));
        elseif(RouteFinal(i,10)==2)
            timerobot(i-1)=max(PT(i-1,4),max(PT(i-1,8),min(PT(i-1,6),PT(i-1,7))));
        elseif(RouteFinal(i,10)==6)
            timerobot(i-1)=max(PT(i-1,4),max(PT(i-1,13),min(PT(i-1,14),PT(i-1,15))));
        else
            timerobot(i-1)=max(PT(i-1,4),min(PT(i-1,14),PT(i-1,15)));
        end
    elseif(i==36)
            timerobot(i-1)=max(PT(i-1,4),min(PT(i-1,14),PT(i-1,15)));
    elseif RouteFinal(i,11)~=0
        if(RouteFinal(i,11)==3) 
            timerobot(i-1)=max(PT(i-1,5),max(PT(i-1,8),min(PT(i-1,9),PT(i-1,10))));
        elseif(RouteFinal(i,11)==4)
            timerobot(i-1)=max(PT(i-1,5),max(min(PT(i-1,11),PT(i-1,12)),min(PT(i-1,9),PT(i-1,10))));
        else
            timerobot(i-1)=max(PT(i-1,5),max(PT(i-1,13),min(PT(i-1,11),PT(i-1,12))));
        end
    end
end
for i=1:35
    result(i,1:4)=ActFinal(i,1:4);
    result(i,5)=timerobot(i);
end
result(32,2)=12;
result(33,2)=12;
result(35,1:2)=[10,12];
result(25,2)=12;
result(26,2)=12;
[~,I]=sort(result(:,5));
result=result(I,:);
toc



