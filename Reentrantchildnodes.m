%% �ҵ��Ӵ�״̬��
function [S,Act]=Reentrantchildnodes(s,Readytime)
%sΪ״̬����10~11λ��ʾ����Ĺ���Ҫ�ŵ�λ�ã�12~13λ��ʾact
%% �ҵ��Ӵ�״̬��
M=9;     % �ӹ��������
RM=12;
% s=[0,0,0,0,0,2,1,0,0,0,0,0,17];
% Readytime=zeros(1,RM+4);
% Readytime(11)=120;
% Readytime(12:13)=[100 100];
% Readytime(14)=120;
A=zeros(M,M);
for i=1:M-1
    A(i,i)=-1;
    A(i,i+1)=1;
end
A(M,M)=-1;
tr1=1;% ȡƬʱ��
tr2=1;% ��Ƭʱ��
tr3=1;% ��תʱ��
% ����ʱ��
MaxS=[5,3,1,1,1,2,1,1,1];%��ʾÿ������������ɹ�������
Act=zeros(1,2);%act=i��ʾ�ӹ���ǻi-1������ǻi������state��i��i+1
Act2=zeros(1,2);
z=zeros(1,M+2); %zi�ӵ�i-1��ǻ���𹤼��ŵ���i��ǻ,���2��Ϊbufferλ��1��ʾ˫���Ѿ�����һ��������0��ʾ��
F1=[find(s(1:2)) find(s(8:M))+7]; % Ѱ�����в�Ϊ0��λ��
sF2=s;
if((s(5)+s(7))==1)
    sF2(5)=1;
    sF2(7)=1;
end
F2=find(sF2(3:8))+2;
Len1=length(F1);
Len2=length(F2);
flag1=0; % flag=0��buffer,1��ʾbuffer1���Էţ�2��ʾ��buffer�����ܷ�
flag2=0; % flag=0��buffer,1��ʾbuffer2���Էţ�2��ʾ��buffer�����ܷ�

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
        if(s(M+1)==5 || s(M+1)==7)
            if((s(5)+s(7))<MaxS(s(M+1)))
                %��buffer1����
                flag1=1;
            else
                flag1=2;
            end
        else
            if(s(s(M+1))<MaxS(s(M+1)))
                %��buffer1����
                flag1=1;
            else
                flag1=2;
            end
        end
    end
    if(s(M+2)>0)
        if(s(M+2)==5 || s(M+2)==7)
            if((s(5)+s(7))<MaxS(s(M+2)))
                flag2=1;
            else
                flag2=2;
            end
        else
            if(s(s(M+2))<MaxS(s(M+2)))
                %��buffer2����
                flag2=1;
            else
                flag2=2;
            end
        end
    end
end

z1=[find(z(1:2)) find(z(8:M))+7];
Z=zeros(1,M); % ��������    
k2=2; % �Ӵ����
S2(1,:)=s(1,:); % S2Ϊ����̨2�����п���״̬
k=2; % �Ӵ����
S(1,1:M+4)=s(1,1:M+4); % SΪ�����Ӵ�
if(flag1==1)
    %��buffer1
    S2(k2,:)=s;
    S2(k2,s(M+1))=s(s(M+1))+1;
    S2(k2,M+1)=0;
    Act2(k2,2)=10+s(M+1);
    k2=k2+1;
end

if(flag2==1)
    %��buffer2
    S2(k2,:)=s;
    S2(k2,s(M+2))=s(s(M+2))+1;
    S2(k2,M+2)=0;
    Act2(k2,2)=10+s(M+2);
    k2=k2+1;
end

minP=zeros(1,M+2);  %ǰ����λR1R2
minP(2)=min(Readytime(6),min(Readytime(7),Readytime(8)));
minP(3)=Readytime(9);   %BM11
minP(4)=Readytime(10);  %PM21
minP(5)=Readytime(11);  %PM22
minP(6)=min(Readytime(12),Readytime(13));
minP(7)=Readytime(14);  %PM22
minP(8)=Readytime(15);  %BM22
minP(9)=Readytime(16);  %PM14

maxP=zeros(1,M+2);  %ǰ����λR1R2
maxP(2)=max(Readytime(6),max(Readytime(7),Readytime(8)));
maxP(3)=Readytime(9);   %BM11
maxP(4)=Readytime(10);  %PM21
maxP(5)=Readytime(11);  %PM22
maxP(6)=max(Readytime(12),Readytime(13));
maxP(7)=Readytime(14);  %PM22
maxP(8)=Readytime(15);  %BM22
maxP(9)=Readytime(16);  %PM14
buffer=[];
bufferi=1;
if(flag1~=1 && flag2~=1)
    %���û��buffer���Է�
    if(Len2>1)
        for F2i=Len2:-1:2
            if(F2(F2i)-F2(F2i-1)==1)
                %���Ϊ2������Ϊ1�� 
                if(F2(F2i)==5 ||F2(F2i)==7)
                    if(s(F2(F2i-1))==MaxS(F2(F2i-1)) && (s(5)+s(7))==MaxS(F2(F2i)) && minP(F2(F2i))>minP(F2(F2i-1)))
                        buffer(bufferi)=F2(F2i-1);
                        bufferi=bufferi+1;
                    elseif(s(F2(F2i-1))<MaxS(F2(F2i-1)) && (s(5)+s(7))==MaxS(F2(F2i)) && minP(F2(F2i))>maxP(F2(F2i-1)))
                        buffer(bufferi)=F2(F2i-1);
                        bufferi=bufferi+1;
                    end
                else
                    if(s(F2(F2i-1))==MaxS(F2(F2i-1)) && s(F2(F2i))==MaxS(F2(F2i)) && minP(F2(F2i))>minP(F2(F2i-1)))
                        buffer(bufferi)=F2(F2i-1);
                        bufferi=bufferi+1;
                    elseif(s(F2(F2i-1))<MaxS(F2(F2i-1)) && s(F2(F2i-1))~=0 && s(F2(F2i))==MaxS(F2(F2i)) && minP(F2(F2i))>maxP(F2(F2i-1)))
                        buffer(bufferi)=F2(F2i-1);
                        bufferi=bufferi+1;
                    end
                end
            end
        end
%         if(~isempty(buffer) && flag1==0 && flag2==0)
        if(~isempty(buffer))
            %��е˫���п��У�Ҫץȡ������Ϊbuffer
            for bufferi2=1:bufferi-1
                S2(k2,:)=s;
                S2(k2,buffer(bufferi2))=S2(k2,buffer(bufferi2))-1;
                if(flag1==0&&flag2==0)
                    S2(k2,M+1)=buffer(bufferi2)+1;
                    Act2(k2,2)=20+buffer(bufferi2);
                    k2=k2+1;
                elseif(flag1~=0)
                    if(buffer(bufferi2)==s(M+1)||buffer(bufferi2)==7)
                        S2(k2,M+2)=buffer(bufferi2)+1;
                        Act2(k2,2)=20+buffer(bufferi2);
                        k2=k2+1;
                    end
                else
                    if(buffer(bufferi2)==s(M+2)||buffer(bufferi2)==7)
                        S2(k2,M+1)=buffer(bufferi2)+1;                        
                        Act2(k2,2)=20+buffer(bufferi2);
                        k2=k2+1;
                    end
                end
            end
        end      
    end
    if(Len2>0 && F2(Len2)==8)
        Len2=Len2-1;
    end
    if(Len2>0)
        %��Ϊ���۹���
        for F2i=1:Len2 
            if(F2(F2i)==4 ||F2(F2i)==6)
                if(s(F2(F2i))>0 && (s(5)+s(7))<MaxS(F2(F2i)+1))
                    Z(F2(F2i))=1;
                    S2(k2,:)=s;
                    S2(k2,1:M)=s(1,1:M)+Z*A;
                    Act2(k2,2)=F2(F2i);
                    k2=k2+1;
                    Z=zeros(1,M);
                end
            else
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
end


if(~isempty(z1))
    %��е��1����
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
    %��е��2����
    for Si=2:k2-1
        S(k,:)=S2(Si,:);
        Act(k,1)=0;
        Act(k,2)=Act2(Si,2);
        k=k+1;
    end

end
S(1,:)=[];
Act(1,:)=[];
S(:,M+3:M+4)=Act(:,1:2);
