function [S,Act]=childnodesnew4(s,Readytime)
%sΪ״̬���󣬹�15λ��12~13λ��ʾ����Ĺ���Ҫ�ŵ�λ�ã�14~15λ��ʾact
%SΪ״̬���󣬹�15λ��12~13λ��ʾ����Ĺ���Ҫ�ŵ�λ�ã�14~15λ��ʾact
M=11;     % �ӹ��������
A=zeros(M,M);
for i=1:M-1
    A(i,i)=-1;
    A(i,i+1)=1;
end
A(M,M)=-1;
% ����ʱ��
Act=zeros(1,2);%act=i��ʾ�ӹ���ǻi-1������ǻi������state��i��i+1
Act2=zeros(1,2);
z=zeros(1,M+2); %zi�ӵ�i-1��ǻ���𹤼��ŵ���i��ǻ,���2��Ϊbufferλ��1��ʾ˫���Ѿ�����һ��������0��ʾ��
F11=find(s(1:3)); % Ѱ�����в�Ϊ0��λ��
F12=find(s(9:M))+8;
F1=[F11 F12];
F2=find(s(4:9))+3;
Len1=length(F1);
Len2=length(F2);
flag1=0; % flag=0��buffer,1��ʾbuffer1���Էţ�2��ʾ��buffer�����ܷ�
flag2=0; % flag=0��buffer,1��ʾbuffer2���Էţ�2��ʾ��buffer�����ܷ�

if(Len1>0)
    for i=1:Len1
         if(F1(i)==M)
         z(M)=1;
        else
            if(s(F1(i)+1)==0)
                z(F1(i))=1;
            end
         end
    end
end

if(Len2>0)
    if(s(M+1)>0)
        if(s(s(M+1))==0)
            %��buffer1����
            flag1=1;
        else
            flag1=2;
        end
    end
    if(s(M+2)>0)
        if(s(s(M+2))==0)
            %��buffer2����
            flag2=1;
        else
            flag2=2;
        end
    end
end

z1=[find(z(1:4)) find(z(9:11))+8];
Z=zeros(1,M); % ��������    
k2=2; % �Ӵ����
S2(1,1:M+2+2)=s(1,1:M+2+2); % S2Ϊ����̨2�����п���״̬
k=2; % �Ӵ����
S(1,1:M+2+2)=s(1,1:M+2+2); % SΪ�����Ӵ�
if(flag1==1)
    %��buffer1
    S2(k2,:)=s;
    S2(k2,s(M+1))=1;
    S2(k2,M+1)=0;
    Act2(k2,2)=10+s(M+1);
    k2=k2+1;
end

if(flag2==1)
    %��buffer2
    S2(k2,:)=s;
    S2(k2,s(M+2))=1;
    S2(k2,M+2)=0;
    Act2(k2,2)=10+s(M+2);
    k2=k2+1;
end

buffer=[];
bufferi=1;
if(flag1~=1 && flag2~=1)
    %���û��buffer���Է�
    if(Len2>1)
        for F2i=Len2:-1:2
            if(F2(F2i)-F2(F2i-1)==1)
                %���Ϊ2������Ϊ1��
                if(Readytime(F2(F2i)+4)>Readytime(F2(F2i-1)+4))
                    buffer(bufferi)=F2(F2i-1);
                    bufferi=bufferi+1;
                end
            end
        end
%         if(~isempty(buffer) && flag1==0 && flag2==0)
        if(~isempty(buffer))
            %��е˫���п��У�Ҫץȡ������Ϊbuffer
            for bufferi2=1:bufferi-1
                S2(k2,:)=s;
                S2(k2,buffer(bufferi2))=0;
                if(flag1==0&&flag2==0)
                    S2(k2,M+1)=buffer(bufferi2)+1;
                    Act2(k2,2)=20+buffer(bufferi2);
                    k2=k2+1;
                elseif(flag1~=0)
                    if(buffer(bufferi2)==s(12)||buffer(bufferi2)==8)
                        S2(k2,M+2)=buffer(bufferi2)+1;
                        Act2(k2,2)=20+buffer(bufferi2);
                        k2=k2+1;
                    end
                else
                    if(buffer(bufferi2)==s(13)||buffer(bufferi2)==8)
                        S2(k2,M+1)=buffer(bufferi2)+1;                        
                        Act2(k2,2)=20+buffer(bufferi2);
                        k2=k2+1;
                    end
                end
            end
        end      
    end
    if(Len2>0 && F2(Len2)==9)
        Len2=Len2-1;
    end
    if(Len2>0)
        %��Ϊ���۹���
        for F2i=1:Len2            
            if(s(F2(F2i))==1 && s(F2(F2i)+1)==0)
                Z(F2(F2i))=1;
                S2(k2,:)=s;
                S2(k2,1:M)=s(1,1:M)+Z*A;
                Act2(k2,2)=F2(F2i);
                k2=k2+1;
                Z=zeros(1,11);
            end
        end
    end
end


if(~isempty(z1))
    %ֻ�л�е��1�в���
    for i=1:length(z1)
        Z(z1(i))=1;
%         Z(z1(i)-1)=1;
        S(k,1:M)=s(1,1:M)+Z*A;
        S(k,M+1:M+2)=s(1,M+1:M+2);
        Z=zeros(1,M);
        Act(k,1)=z1(i);
        Act(k,2)=0;
        k=k+1;
    end
end
if(k2>2)
    %��е��2�в���
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
end