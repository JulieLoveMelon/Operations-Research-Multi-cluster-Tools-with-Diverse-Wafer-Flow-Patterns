%% 串行
clear all;
tic
%% 初始参数
RM=12;   % 腔室个数
M=9;     % 加工工序个数
s=zeros(1,M+2+2);   % 状态机,每个工作腔中工件个数,8~9位为buffer,10~11位表示act
MaxS=[5,3,1,1,1,2,1,1,1];%表示每个工序最多容纳工件个数
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
PMTime=[0,200,0,60,50,130,50,0,110];
% PMTime=[0,70,0,50,60,0,50];
Readytime=zeros(1,RM+4);%RT[1]为父代序号，2为子代序号，3为父代层数，4，5是Robot时间，6-15是10道工序时间
Readytime(2)=1;
Makespan=10000;
% z=zeros(1,M); %zi从第i-1个腔拿起工件放到第i个腔
%% 可重入
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
        [child,act]=Reentrantchildnodes(P(i,:),PReadytime);
        [CH,CL]=size(child);
         minP=zeros(2,M+2); 
         maxP=zeros(2,M+2);
         midP=zeros(2,1);
         if Readytime(k,6)<=Readytime(k,7) && Readytime(k,6)<=Readytime(k,8)
             minP(1,2)=Readytime(k,6);
             minP(2,2)=3;
             if Readytime(k,7)<=Readytime(k,8)
                maxP(1,2)=Readytime(k,8);
                maxP(2,2)=5;
                midP(1,1)=Readytime(k,7);
                midP(2,1)=4;
             else
                maxP(1,2)=Readytime(k,7);
                maxP(2,2)=4;
                midP(1,1)=Readytime(k,8);
                midP(2,1)=5;
             end
         elseif Readytime(k,7)<=Readytime(k,6) && Readytime(k,7)<=Readytime(k,8)
             minP(1,2)=Readytime(k,7);
             minP(2,2)=4;
             if Readytime(k,6)<=Readytime(k,8)
                maxP(1,2)=Readytime(k,8);
                maxP(2,2)=5;
                midP(1,1)=Readytime(k,6);
                midP(2,1)=3;
             else
                maxP(1,2)=Readytime(k,6);
                maxP(2,2)=3;
                midP(1,1)=Readytime(k,8);
                midP(2,1)=5;
             end
         else
             minP(1,2)=Readytime(k,8);
             minP(2,2)=5;
             if Readytime(k,6)<=Readytime(k,7)
                maxP(1,2)=Readytime(k,7);
                maxP(2,2)=4;
                midP(1,1)=Readytime(k,6);
                midP(2,1)=3;
             else
                maxP(1,2)=Readytime(k,6);
                maxP(2,2)=3;
                midP(1,1)=Readytime(k,7);
                midP(2,1)=4;
             end
         end
         minP(:,3)=[Readytime(k,9); 6];
         maxP(:,3)=[Readytime(k,9); 6];
         minP(:,4)=[Readytime(k,10); 7];
         maxP(:,4)=[Readytime(k,10); 7];
         minP(:,5)=[Readytime(k,11); 8];
         maxP(:,5)=[Readytime(k,11); 8];
                  
         if Readytime(k,12)<=Readytime(k,13)
             minP(1,6)=Readytime(k,12);
             minP(2,6)=9;
             maxP(1,6)=Readytime(k,13);
             maxP(2,6)=10;
         else
             minP(1,6)=Readytime(k,13);
             minP(2,6)=10;
             maxP(1,6)=Readytime(k,12);
             maxP(2,6)=9;
         end     
         minP(:,7)=[Readytime(k,14); 11];
         maxP(:,7)=[Readytime(k,14); 11];
         minP(:,8)=[Readytime(k,15); 12];
         maxP(:,8)=[Readytime(k,15); 12];
         minP(:,9)=[Readytime(k,16); 13];
         maxP(:,9)=[Readytime(k,16); 13];
        for j=1:CH   
            Temptime(j,:)=Readytime(k,4:RM+4);             
               if(act(j,2)~=0)
                if(act(j,2)>=14 && act(j,2)<=18)
                    if(act(j,2)==15 || act(j,2)==17)
                        Temptime(j,2)=max(Readytime(k,5),minP(1,act(j,2)-10))+2;
                        Temptime(j,minP(2,act(j,2)-10))=max(Readytime(k,5),Temptime(j,minP(2,act(j,2)-10)))+PMTime(act(j,2)-10)+1;
                        Temptime(j,8)=max(Temptime(j,8),Temptime(j,11));
                        Temptime(j,11)=max(Temptime(j,8),Temptime(j,11));
                    else
                    %双臂放buffer
                        Temptime(j,2)=max(Readytime(k,5),minP(1,act(j,2)-10))+2;
                        Temptime(j,minP(2,act(j,2)-10))=max(Readytime(k,5),Temptime(j,minP(2,act(j,2)-10)))+PMTime(act(j,2)-10)+1;
                    end
                elseif(act(j,2)>=23 && act(j,2)<=27)
                    %取buffer
                    if(act(j,2)==25 || act(j,2)==27)
                        Temptime(j,2)=max(Readytime(k,5),minP(1,act(j,2)-20))+2;
                        Temptime(j,minP(2,act(j,2)-20))=max(Readytime(k,5),Temptime(j,minP(2,act(j,2)-20)))+1;
                        Temptime(j,8)=max(Temptime(j,8),Temptime(j,11));
                        Temptime(j,11)=max(Temptime(j,8),Temptime(j,11));
                    else
                        if(P(i,act(j,2)-20)==MaxS(act(j,2)-20))
                            Temptime(j,2)=max(Readytime(k,5),minP(1,act(j,2)-20))+2;
                            Temptime(j,minP(2,act(j,2)-20))=max(Readytime(k,5),Temptime(j,minP(2,act(j,2)-20)))+1;
                        else
                            Temptime(j,2)=max(Readytime(k,5),maxP(1,act(j,2)-20))+2;
                            Temptime(j,maxP(2,act(j,2)-20))=max(Readytime(k,5),Temptime(j,maxP(2,act(j,2)-20)))+1;
                        end
                    end
                else %作为单臂使用
                    if(act(j,2)==4 || act(j,2)==6)
                        if(P(i,act(j,2))==MaxS(act(j,2)))
                            Temptime(j,2)=max(Readytime(k,5),max(minP(1,act(j,2)),minP(1,act(j,2)+1)))+4;
                            Temptime(j,minP(2,act(j,2)+1))=max(Readytime(k,5),max(minP(1,act(j,2)),minP(1,act(j,2)+1)))+PMTime(act(j,2)+1)+3;
                            Temptime(j,8)=max(Temptime(j,8),Temptime(j,11));
                            Temptime(j,11)=max(Temptime(j,8),Temptime(j,11));
                        else
                            Temptime(j,2)=max(Readytime(k,5),max(maxP(1,act(j,2)),minP(1,act(j,2)+1)))+4;
                            Temptime(j,minP(2,act(j,2)+1))=max(Readytime(k,5),max(maxP(1,act(j,2)),minP(1,act(j,2)+1)))+PMTime(act(j,2)+1)+3;
                            Temptime(j,8)=max(Temptime(j,8),Temptime(j,11));
                            Temptime(j,11)=max(Temptime(j,8),Temptime(j,11));
                        end
                        
                        
                    else
                        if(P(i,act(j,2))==MaxS(act(j,2)))
                            Temptime(j,2)=max(Readytime(k,5),max(minP(1,act(j,2)),minP(1,act(j,2)+1)))+4;
                            Temptime(j,minP(2,act(j,2)+1))=max(Readytime(k,5),max(minP(1,act(j,2)),minP(1,act(j,2)+1)))+PMTime(act(j,2)+1)+3;
                        else
                            Temptime(j,2)=max(Readytime(k,5),max(maxP(1,act(j,2)),minP(1,act(j,2)+1)))+4;
                            Temptime(j,minP(2,act(j,2)+1))=max(Readytime(k,5),max(maxP(1,act(j,2)),minP(1,act(j,2)+1)))+PMTime(act(j,2)+1)+3;
                        end
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
                elseif(act(j,1)==2)
                    if(P(i,act(j,1))==MaxS(act(j,1)))
                        Temptime(j,1)=max(Readytime(k,4),max(minP(1,act(j,1)),minP(1,act(j,1)+1)))+4;
                        Temptime(j,minP(2,act(j,1)+1))=max(Readytime(k,4),max(minP(1,act(j,1)),minP(1,act(j,1)+1)))+PMTime(act(j,1)+1)+3;
                    elseif(P(i,act(j,1))==MaxS(act(j,1))-1)
                        Temptime(j,1)=max(Readytime(k,4),max(midP(1,1),maxP(1,act(j,1)+1)))+4;
                        Temptime(j,minP(2,act(j,1)+1))=max(Readytime(k,4),max(midP(1,1),minP(1,act(j,1)+1)))+PMTime(act(j,1)+1)+3;
                    
                    else
                        Temptime(j,1)=max(Readytime(k,4),max(maxP(1,act(j,1)),maxP(1,act(j,1)+1)))+4;
                        Temptime(j,minP(2,act(j,1)+1))=max(Readytime(k,4),max(maxP(1,act(j,1)),minP(1,act(j,1)+1)))+PMTime(act(j,1)+1)+3;
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
              for m=3:5
                  Temptime(j,m)=max(Temptime(j,1),Temptime(j,m));
              end
              for m=13
                  Temptime(j,m)=max(Temptime(j,1),Temptime(j,m));
              end
              for m=7:11
                  Temptime(j,m)=max(Temptime(j,2),Temptime(j,m));
              end
              Temptime(j,6)=max(Temptime(j,6),min(Temptime(j,1),Temptime(j,2)));
              Temptime(j,12)=max(Temptime(j,12),min(Temptime(j,1),Temptime(j,2)));
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
            minTempReadyTime(tempmini,4)=min(TempReadyTime(tempmini,4),min(TempReadyTime(tempmini,5),TempReadyTime(tempmini,6)));
            minTempReadyTime(tempmini,5)=minTempReadyTime(tempmini,4);
            minTempReadyTime(tempmini,6)=minTempReadyTime(tempmini,4);
            minTempReadyTime(tempmini,7)=TempReadyTime(tempmini,7);
            minTempReadyTime(tempmini,8)=TempReadyTime(tempmini,8);
            minTempReadyTime(tempmini,9)=TempReadyTime(tempmini,9);
            minTempReadyTime(tempmini,10)=min(TempReadyTime(tempmini,11),TempReadyTime(tempmini,10));
            minTempReadyTime(tempmini,11)=minTempReadyTime(tempmini,10);
            minTempReadyTime(tempmini,12)=TempReadyTime(tempmini,12);
            minTempReadyTime(tempmini,13)=TempReadyTime(tempmini,13);
            minTempReadyTime(tempmini,14)=TempReadyTime(tempmini,14); 
            
            maxTempReadyTime(tempmini,1:3)=TempReadyTime(tempmini,1:3);
            maxTempReadyTime(tempmini,4)=max(TempReadyTime(tempmini,4),max(TempReadyTime(tempmini,5),TempReadyTime(tempmini,6)));
            maxTempReadyTime(tempmini,5)=maxTempReadyTime(tempmini,4);
            maxTempReadyTime(tempmini,6)=maxTempReadyTime(tempmini,4);
            maxTempReadyTime(tempmini,7)=TempReadyTime(tempmini,7);
            maxTempReadyTime(tempmini,8)=TempReadyTime(tempmini,8);
            maxTempReadyTime(tempmini,9)=TempReadyTime(tempmini,9);
            maxTempReadyTime(tempmini,10)=max(TempReadyTime(tempmini,11),TempReadyTime(tempmini,10));
            maxTempReadyTime(tempmini,11)=maxTempReadyTime(tempmini,10);
            maxTempReadyTime(tempmini,12)=TempReadyTime(tempmini,12);
            maxTempReadyTime(tempmini,13)=TempReadyTime(tempmini,13);
            maxTempReadyTime(tempmini,14)=TempReadyTime(tempmini,14); 
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
%% 输出动作序列
RouteFinal=route(end:-1:1,:);
PT=PPP(end:-1:1,:);
ActTemp=RouteFinal(:,end-1:end);
ActFinal(:,1)=ActTemp(:,1);
ActFinal(:,2)=ActTemp(:,1);
ActFinal(:,3)=ActTemp(:,2);
ActFinal(:,4)=ActTemp(:,2);
ActFinal(ActFinal==9)=12;
ActFinal(ActFinal==8)=11;
ActFinal(ActFinal==7)=10;
ActFinal(ActFinal==6)=8;
ActFinal(ActFinal==5)=7;
ActFinal(ActFinal==4)=6;
ActFinal(ActFinal==3)=5;
for i=2:size(ActTemp,1)
     minP=zeros(2,M+2); 
     maxP=zeros(2,M+2);
     midP=zeros(2,1);
     minP(2,1)=1;
     maxP(2,1)=1;
     if PT(i-1,6)<=PT(i-1,7) && PT(i-1,6)<=PT(i-1,8)
         minP(1,2)=PT(i-1,6);
         minP(2,2)=2;
         if PT(i-1,7)<=PT(i-1,8)
            maxP(1,2)=PT(i-1,8);
            maxP(2,2)=4;
            midP(1,1)=PT(i-1,7);
            midP(2,1)=3;
         else
            maxP(1,2)=PT(i-1,7);
            maxP(2,2)=3;
            midP(1,1)=PT(i-1,8);
            midP(2,1)=4;
         end
     elseif PT(i-1,7)<=PT(i-1,6) && PT(i-1,7)<=PT(i-1,8)
         minP(1,2)=PT(i-1,7);
         minP(2,2)=3;
         if PT(i-1,6)<=PT(i-1,8)
            maxP(1,2)=PT(i-1,8);
            maxP(2,2)=4;
            midP(1,1)=PT(i-1,6);
            midP(2,1)=2;
         else
            maxP(1,2)=PT(i-1,6);
            maxP(2,2)=2;
            midP(1,1)=PT(i-1,8);
            midP(2,1)=4;
         end
     else
         minP(1,2)=PT(i-1,8);
         minP(2,2)=4;
         if PT(i-1,6)<=PT(i-1,7)
            maxP(1,2)=PT(i-1,7);
            maxP(2,2)=3;
            midP(1,1)=PT(i-1,6);
            midP(2,1)=2;
         else
            maxP(1,2)=PT(i-1,6);
            maxP(2,2)=2;
            midP(1,1)=PT(i-1,7);
            midP(2,1)=3;
         end
     end
     minP(:,3)=[PT(i-1,9); 5];
     maxP(:,3)=[PT(i-1,9); 5];
     minP(:,4)=[PT(i-1,10);6];
     maxP(:,4)=[PT(i-1,10);6];
     minP(:,5)=[PT(i-1,11);7];
     maxP(:,5)=[PT(i-1,11);7];

     if PT(i-1,12)<=PT(i-1,13)
         minP(1,6)=PT(i-1,12);
         minP(2,6)=8;
         maxP(1,6)=PT(i-1,13);
         maxP(2,6)=9;
     else
         minP(1,6)=PT(i-1,13);
         minP(2,6)=9;
         maxP(1,6)=PT(i-1,12);
         maxP(2,6)=8;
     end

     minP(:,7)=[PT(i-1,14); 10];
     maxP(:,7)=[PT(i-1,14); 10];
     minP(:,8)=[PT(i-1,15); 11];
     maxP(:,8)=[PT(i-1,15); 11];
     minP(:,9)=[PT(i-1,16); 12];
     maxP(:,9)=[PT(i-1,16); 12];
     
     if(ActTemp(i,1)~=0)
       if(RouteFinal(i-1,ActTemp(i,1))<MaxS(ActTemp(i,1)))
           if(RouteFinal(i-1,2)==2&&ActFinal(i,1)==2)
               ActFinal(i,1)=midP(2,1);
           else
               ActFinal(i,1)=maxP(2,ActTemp(i,1));
           end
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
     if(ActTemp(i,2)>=14&&ActTemp(i,2)<=18)
         %放buffer
         ActFinal(i,3)=100;%100表示在机械臂上
         ActFinal(i,4)=minP(2,ActTemp(i,2)-10);
     end
     if(ActTemp(i,2)>=23&&ActTemp(i,2)<=27)
         %取buffer
         ActFinal(i,4)=100;%100表示在机械臂上
         if(RouteFinal(i-1,ActTemp(i,2)-20)<MaxS(ActTemp(i,2)-20))
           ActFinal(i,3)=maxP(2,ActTemp(i,2)-20);
         else
           ActFinal(i,3)=minP(2,ActTemp(i,2)-20);
         end
     end
end
ActFinal(ActFinal==10)=7;
ActFinal(ActFinal==11)=10;
ActFinal(ActFinal==12)=11;
ActFinal(end,1)=11;
ActFinal(1,:)=[];
for i=2:size(PT,1)
    if RouteFinal(i,12)~=0
        if(RouteFinal(i,12)==1)
            timerobot(i-1)=max(PT(i-1,4),min(PT(i-1,8),min(PT(i-1,6),PT(i-1,7))));
        elseif(RouteFinal(i,12)==2)
            if(RouteFinal(i-1,2)==3)
                timerobot(i-1)=max(PT(i-1,9),max(PT(i-1,4),min(PT(i-1,8),min(PT(i-1,6),PT(i-1,7)))));
            elseif(RouteFinal(i-1,2)==1)
                timerobot(i-1)=max(PT(i-1,9),max(PT(i-1,4),max(PT(i-1,8),max(PT(i-1,6),PT(i-1,7)))));
            else
                if((PT(i-1,8)>=PT(i-1,7))&&(PT(i-1,8)<=PT(i-1,6))||(PT(i-1,8)>=PT(i-1,6))&&(PT(i-1,8)<=PT(i-1,7)))
                    timerobot(i-1)=max(PT(i-1,4),PT(i-1,8));
                elseif((PT(i-1,6)>=PT(i-1,7))&&(PT(i-1,6)<=PT(i-1,8))||(PT(i-1,6)>=PT(i-1,8))&&(PT(i-1,6)<=PT(i-1,7)))
                    timerobot(i-1)=max(PT(i-1,4),PT(i-1,6));
                else
                    timerobot(i-1)=max(PT(i-1,4),PT(i-1,7));
                end
            end
        elseif(RouteFinal(i,12)==8)
            timerobot(i-1)=max(PT(i-1,4),max(PT(i-1,15),PT(i-1,16)));
        else
            timerobot(i-1)=max(PT(i-1,4),PT(i-1,16));
        end
    elseif(i==52)
            timerobot(i-1)=max(PT(i-1,4),PT(i-1,16));
    elseif RouteFinal(i,13)~=0
        if(RouteFinal(i,13)==3) 
            timerobot(i-1)=max(PT(i-1,5),max(PT(i-1,9),PT(i-1,10)));
        elseif(RouteFinal(i,13)==4)
            timerobot(i-1)=max(PT(i-1,5),max(PT(i-1,10),PT(i-1,11)));
        elseif(RouteFinal(i,13)==5)
            timerobot(i-1)=max(PT(i-1,5),max(PT(i-1,11),min(PT(i-1,12),PT(i-1,13))));
        elseif(RouteFinal(i,13)==6)
            timerobot(i-1)=max(PT(i-1,5),max(PT(i-1,14),min(PT(i-1,12),PT(i-1,13))));
        elseif(RouteFinal(i,13)==7)
            timerobot(i-1)=max(PT(i-1,5),max(PT(i-1,14),PT(i-1,15)));
        elseif(RouteFinal(i,13)==23)
            timerobot(i-1)=max(PT(i-1,5),PT(i-1,9));
        elseif(RouteFinal(i,13)==24)
            timerobot(i-1)=max(PT(i-1,5),PT(i-1,10));
        elseif(RouteFinal(i,13)==25)
            timerobot(i-1)=max(PT(i-1,5),PT(i-1,11));
        elseif(RouteFinal(i,13)==26)
            timerobot(i-1)=max(PT(i-1,5),min(PT(i-1,12),PT(i-1,13)));       
        elseif(RouteFinal(i,13)==27)
            timerobot(i-1)=max(PT(i-1,5),PT(i-1,14)); 
        elseif(RouteFinal(i,13)==14)
            timerobot(i-1)=max(PT(i-1,5),PT(i-1,10));   
        elseif(RouteFinal(i,13)==15)
            timerobot(i-1)=max(PT(i-1,5),PT(i-1,11)); 
        elseif(RouteFinal(i,13)==16)
            timerobot(i-1)=max(PT(i-1,5),min(PT(i-1,12),PT(i-1,13)));        
        elseif(RouteFinal(i,13)==17)
            timerobot(i-1)=max(PT(i-1,5),PT(i-1,14));   
        else
            timerobot(i-1)=max(PT(i-1,5),PT(i-1,15));
        end
    end
end
for i=1:51
    result(i,1:4)=ActFinal(i,1:4);
    result(i,5)=timerobot(i);
end
result(25,1)=3;
result(38,2)=12;
result(42,2)=12;
result(46,2)=12;
result(49,2)=12;
result(51,2)=12;
[~,I]=sort(result(:,5));
result=result(I,:);
toc



