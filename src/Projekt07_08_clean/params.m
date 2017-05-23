Ypp = 36.37;
dU = 20;
D = 300;
N = 300;
Nu = 300;
lambda = 0.4;

%DMC1: D = 50, N = 50, Nu = 50, lam = 1;
%DMC2: D = 50, N = 50, Nu = 50, lam = 0.5;
%DMC3: D = 50, N = 50, Nu = 50, lam = 0.2; fail
%DMC4: D = 50, N = 50, Nu = 10, lam = 0.5; best
%DMC5: D = 50, N = 10, Nu = 10, lam = 0.5; fail
%DMC6: D = 50, N = 20, Nu = 10, lam = 0.5; fail
%DMC7: D = 50, N = 30, Nu = 10, lam = 0.5; fail

load step.mat
s = (y(13:end) - Ypp)/dU;


M=zeros(N,Nu);
for i=1:N
   for j=1:Nu
      if (i>=j)
         M(i,j)=s(i-j+1);
      end
   end
end

Mp=zeros(N,D-1);
for i=1:N
   for j=1:D-1
      if i+j<=D
         Mp(i,j)=s(i+j)-s(j);
      else
         Mp(i,j)=s(D)-s(j);
      end
   end
end

K=((M'*M+lambda*eye(Nu))^-1)*M';
Ku=K(1,:)*Mp;
Ke=sum(K(1,:));

fprintf(strcat('    static float Ke = ',sprintf('%.4f',Ke),';\n',...
    '    static float Ku[] = {',sprintf('%.4f,',Ku),'\b};\n'));