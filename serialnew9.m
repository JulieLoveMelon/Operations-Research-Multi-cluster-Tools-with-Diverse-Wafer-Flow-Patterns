%% 串行
clear all;
tic
%% 初始参数
M=11;     % 加工工序个数
s=zeros(1,M+2+2);   % 状态机,每个工作腔中工件个数，第12~13列表示缓存的工件要放的位置，14~15位表示act
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
PMTime=[60,70,0,40,50,50,60,0,50,48];
% PMTime=[139,139,0,131,131,130,130,0,120,120];
Readytime=zeros(1,M+1+3);%RT[1]为父代序号，2为子代序号，3为父代层数，4，5是Robot时间，6-15是10道工序时间
Readytime(2)=1;
Makespan=10000;
% z=zeros(1,M); %zi从第i-1个腔拿起工件放到第i个腔
%% 串行
Parent={s};
Generation=1;
Readym=2;
Allnodes=zeros(1,M+6);%所有节点，用于比较查重，第一列父节点所在层数，第二列为自己在自己层数内序号，之后为状态机，16~17位表示act
Allnodes(1)=1;Allnodes(2)=1;Allnodes(3)=5;
allnodesm=2;
while(1)
    P=Parent{Generation};
    [PH,PL]=size(P);
    TempReadyTime=zeros(1,M+2+M+2+2);% 存同一层父代所有的子代的readytime,后15位为状态机
    TRTm=1;%Tempreadytime编号
    for i=1:PH
        Temptime=zeros(1,M+1);%存取子代readytime.1,2为robot时间
        k=find(Readytime(:,2)==i & Readytime(:,3)==(Generation-1));
        PReadytime=Readytime(k,:);
        [child,act]=childnodesnew4(P(i,:),PReadytime);
        [CH,CL]=size(child);    
         for j=1:CH             
             Temptime(j,:)=Readytime(k,4:M+4);
              if(act(j,2)~=0)
                if(act(j,2)>13 && act(j,2)<=19)
                    %双臂放buffer
                    Temptime(j,2)=max(Readytime(k,5),Readytime(k,act(j,2)-10+5))+2;
                    %Temptime(j,act(j,2)-8)=Temptime(j,act(j,2)-8)+PMTime(act(j,2)-10)+1;
                    Temptime(j,act(j,2)-9)=max(Readytime(k,5),Temptime(j,act(j,2)-9))+PMTime(act(j,2)-11)+1;
                elseif(act(j,2)>23 && act(j,2)<=29)   
                    %取buffer
                    Temptime(j,2)=max(Readytime(k,5),Temptime(j,act(j,2)-19))+2;
                    Temptime(j,act(j,2)-19)=max(Readytime(k,5),Temptime(j,act(j,2)-19))+1;
                else %作为单臂使用
                    Temptime(j,2)=max(Readytime(k,5),max(Readytime(k,act(j,2)+4),Readytime(k,act(j,2)+5)))+4;
                    Temptime(j,act(j,2)+2)=max(Readytime(k,5),max(Readytime(k,act(j,2)+4),Readytime(k,act(j,2)+5)))+PMTime(act(j,2))+3;
                end
              end
              if(act(j,1)~=0)
                if(act(j,1)==1)
                    Temptime(j,1)=max(Readytime(k,4),Readytime(k,4))+4;
                    Temptime(j,3)=max(Readytime(k,4),Readytime(k,4))+PMTime(1)+3;
                elseif(act(j,1)==11)
                    Temptime(j,1)=max(Readytime(k,4),Readytime(k,15))+4;
                    Temptime(j,M+1)=max(Readytime(k,4),Readytime(k,15))+1;
                else
                    Temptime(j,1)=max(Readytime(k,4),max(Readytime(k,act(j,1)+4),Readytime(k,act(j,1)+5)))+4;
                    Temptime(j,act(j,1)+2)=max(Readytime(k,4),max(Readytime(k,act(j,1)+4),Readytime(k,act(j,1)+5)))+PMTime(act(j,1))+3; 
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
              TempReadyTime(TRTm,2:M+2)=Temptime(j,:);
              TempReadyTime(TRTm,1)=i; %TempReadyTime的第一列为当前子代们的父代的编号
              TempReadyTime(TRTm,M+3:end)=child(j,:);
              TRTm=TRTm+1;
        end
    end
    %%  找到Tempreadytime中的相同子代，删去被支配的路径
        Tag=ones(1,TRTm-1);
        [~,diffm,diffn]=unique(TempReadyTime(:,M+3:26),'rows'); %diffm为含有不同元素的行号，diffn为编号
        if(size(diffm,1)~=TRTm-1)
            %有相同子代
            for i=1:max(diffn)
                sameh=find(diffn==i);
                if(size(sameh,1)>1)    
                %    if(TempReadyTime(sameh(1),14:25)==0)
                   % else
                        for j=1:size(sameh,1)-1
                            for n=j+1:size(sameh,1)
                                if(all(TempReadyTime(sameh(j),2:M+2)>=TempReadyTime(sameh(n),2:M+2)))
                                    Tag(sameh(j))=0;
                                elseif(all(TempReadyTime(sameh(j),2:M+2)<=TempReadyTime(sameh(n),2:M+2)))
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
        Tempchild=zeros(1,M+4);
        m=1;
        for j=1:length(remain)
            Readytime(Readym,1)=TempReadyTime(remain(j),1);
            Readytime(Readym,2)=j;
            Readytime(Readym,3)=Generation;
            Readytime(Readym,4:M+4)=TempReadyTime(remain(j),2:13);
            Tempchild(m,1:M+4)=TempReadyTime(remain(j),14:end);
            Readym=Readym+1;
            Allnodes(allnodesm,1)=Generation;
            Allnodes(allnodesm,2)=j;
            Allnodes(allnodesm,3:M+6)=TempReadyTime(remain(j),14:end);
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
    if(Allnodes(i,3:15)==0)
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

%% 输出动作序列与对应时间
% Z=zeros(1,M);
% Action=zeros(size(route,1)-1,2);
% for i=1:size(route,1)-1
%     routediff=route(i,1:M)-route(i+1,1:M);
%     if(isequal(route(i,M+1:M+2),route(i+1,M+1:M+2)))
%         Z=(route(i,1:M)-route(i+1,1:M))*A^(-1); 
%         Act1=[find(Z(1:3)==1) find(Z(9:11)==1)+8];
%         Act2=find(Z(4:8))+3;
%         if(Act1)
%             Action(i,1)=Act1;
%         end
%         if(Act2)
%             Action(i,2)=Act2;
%         end
%     else
%         if(isequal(route(i,M+1:M+2),[0 0]))%放一个buffer
%             Action(i,2)=10+max(route(i+1,M+1),route(i+1,M+2));
%             routediff(max(route(i+1,M+1),route(i+1,M+2)))=routediff(max(route(i+1,M+1),route(i+1,M+2)))-1;
%             Z=routediff*A^(-1);
%             Act1=[find(Z(1:3)==1) find(Z(9:11)==1)+8];
%             if(Act1)
%                 Action(i,1)=Act1;
%             end
%         elseif(isequal(route(i+1,M+1:M+2),[0 0]))%取一个buffer
%             Action(i,2)=20+max(route(i,M+1),route(i,M+2))-1;
%             routediff(max(route(i,M+1),route(i,M+2))-1)=routediff(max(route(i,M+1),route(i,M+2))-1)+1;
%             Z=routediff*A^(-1);
%             Act1=[find(Z(1:3)==1) find(Z(9:11)==1)+8];
%             if(Act1)
%                 Action(i,1)=Act1;
%             end
%         elseif((route(i,M+1)-route(i+1,M+1)<=0) &&(route(i,M+2)-route(i+1,M+2)<=0)) %放一个buffer
%             if(route(i,M+1)-route(i+1,M+1)<0)%放下的是M+1位的buffer
%                 Action(i,2)=10+route(i+1,M+1)-route(i,M+1);
%                 routediff(route(i+1,M+1))=routediff(route(i+1,M+1))-1;
%                 Z=routediff*A^(-1);
%                 Act1=[find(Z(1:3)==1) find(Z(9:11)==1)+8];
%                 if(Act1)
%                     Action(i,1)=Act1;
%                 end
%             elseif(route(i,M+2)-route(i+1,M+2)<0)%放下的是M+2位的buffer
%                 Action(i,2)=10+route(i+1,M+2)-route(i,M+2);
%                 routediff(route(i,M+2))=routediff(route(i,M+2))-1;
%                 Z=routediff*A^(-1);
%                 Act1=[find(Z(1:3)==1) find(Z(9:11)==1)+8];
%                 if(Act1)
%                     Action(i,1)=Act1;
%                 end
%             end
%         else%成双buffer
%             if(route(i,M+1)-route(i+1,M+1)>0)%加入的是M+1位的buffer
%                 Action(i,2)=20+route(i,M+1)-route(i+1,M+1)-1;
%                 routediff(route(i,M+1)-1)=routediff(route(i,M+1)-1)+1;
%                 Z=routediff*A^(-1);
%                 Act1=[find(Z(1:3)==1) find(Z(9:11)==1)+8];
%                 if(Act1)
%                     Action(i,1)=Act1;
%                 end
%             elseif(route(i,M+2)-route(i+1,M+2)>0)%加入的是M+2位的buffer
%                 Action(i,2)=20+route(i,M+2)-route(i+1,M+2)-1;
%                 routediff(route(i,M+2)-1)=routediff(route(i,M+2)-1)+1;
%                 Z=routediff*A^(-1);
%                 Act1=[find(Z(1:3)==1) find(Z(9:11)==1)+8];
%                 if(Act1)
%                     Action(i,1)=Act1;
%                 end
%             end
%         end   
%     end
% end
timefinal=time(end:-1:1);
routefinal=route(end:-1:1,:);
% Actionfinal=Action(end:-1:1,:);
PT=PPP(end:-1:1,:);

for i=2:size(PT,1)
    if routefinal(i,14)~=0
        if(routefinal(i,14)==1)
            timerobot(i-1)=max(PT(i-1,4),PT(i-1,6));
        elseif(routefinal(i,14)==11)
            timerobot(i-1)=max(PT(i-1,4),PT(i-1,15));
        else
             timerobot(i-1)=max(PT(i-1,4),max(PT(i-1,routefinal(i,14)+5),PT(i-1,routefinal(i,14)+4)));
        end
    elseif(i==56)
            timerobot(i-1)=max(PT(i-1,4),PT(i-1,15));
    elseif routefinal(i,15)~=0
        timerobot(i-1)=max(PT(i-1,5),max(PT(i-1,routefinal(i,14)+5),PT(i-1,routefinal(i,14)+4)));
    end
end
for i=1:55
    result(i,1:2)=routefinal(i+1,14:15);
    result(i,3)=timerobot(i);
end
result(55,1)=11;
[~,I]=sort(result(:,3));
result=result(I,:);
toc



