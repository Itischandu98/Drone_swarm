close all;
clear all;
clc;

nobs=17;
initial=randi(30,nobs,2)
problems=[0 0; 0 15; 15 7.5; 20 20]; %Random points to swarm around
pc=col(problems);
n=col(initial);
sets=floor(n/pc);
bots=[];

for i=1:1:pc
    final=[problems(i,1) problems(i,2)];
    sol=dist(final,initial);
    [buffer,index]=sort(sol);
    index(ismember(index,[bots]))=[];
    if rem(n,pc)>=i
        bots=[bots,index(1:sets+1)]
    else
        bots=[bots,index(1:sets),0]
    end
end


for i=1:1:pc
    reqpointsforsim=problem(i,sets+1,bots)
    [p,q]=size(reqpointsforsim);
    shape(q) + problems(i,:)
    DroneMotion(q,initial(reqpointsforsim,:),problems(i,:),shape(q)+problems(i,:))
end

function pro=problem(i,pc,bots)
    temp=(i-1)*pc;
    out=bots(temp+1:temp+pc);
    pro=out(out~=0);
end
    
function columns=col(input)
    sze=size(input);
    columns=sze(1);
end

function distance=dist(final,initial)
    temp=[];
    cols=size(initial);
    for i=1:1:cols(1)
        temp(end+1)=sqrt((final(1)-initial(i,1))^2+(final(2)-initial(i,2))^2);
    end
    distance=temp;
end