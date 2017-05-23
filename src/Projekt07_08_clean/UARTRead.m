delete(instrfindall); % zamkniecie wszystkich polaczen szeregowych
clear all;
s = serial('COM18'); % COM9 to jest port utworzony przez mikrokontroler
set(s,'BaudRate',115200);
set(s,'StopBits',1);
set(s,'Parity','none');
set(s,'DataBits',8);
set(s,'Timeout',3);
set(s,'InputBufferSize',1000);
set(s,'Terminator',13);
fopen(s); % otwarcie kanalu komunikacyjnego

Tp = 1; % czas z jakim probkuje regulator
y = [];    % wektor wyjsc obiektu
yzad = []; % wektor trajektorii zadanej
u = [];    % wektor wejsc (sterowan) obiektu
step = 0;
% y(1:50)=0;
% u(1:50)=0;
% yzad(1:50)=0;
% Yzad_ = 100;
% while ~step
%     txt = fread(s,36);
%     
%     eval(char(txt'));
%     if Yzad > 0 && Yzad_ == 0
%         step = 1;
%     end
%     y(1:49)=y(2:50);
%     y(50)=Y;
%     u(1:49)=u(2:50);
%     u(50)=U;
%     yzad(1:49)=yzad(2:50);
%     yzad(50)=Yzad;
%     Yzad_ = Yzad;
% end
% 
% for i = 1:150
% %while length(y)~=150    % zbieramy 100 pomiarow
%     txt = fread(s,36);  % odczytanie z portu szeregowego
%                         % txt powinien zawieraæ "Y=%4d;U=%4d;"
%                         % czyli np. "Y=1234;U=3232;"
% %    disp(char(txt'));
%     eval(char(txt'));   % wykonajmy to co otrzymalismy
%     y=[y Y];            % powiekszamy wektor y o element Y
%     u=[u U];            % powiekszamy wektor u o element U
%     yzad=[yzad Yzad];
% end
figure;
for i = 1:400
    txt = fread(s,50);
    disp(char(txt'))
    eval(char(txt'));
    y=[y Y];            % powiekszamy wektor y o element Y
    u=[u U];            % powiekszamy wektor u o element U
    yzad=[yzad Yzad];
    subplot(2,1,1);
    plot(y);
    hold on;
    plot(yzad,':');
    hold off;
    subplot(2,1,2);
    plot(u+50);
    pause(0.01);
end
    
figure; plot((1:(length(y)))*Tp,y); % wyswietlamy y w czasie
figure; plot((1:(length(y)))*Tp,yzad); % wyswietlamy y w czasie
figure; plot((1:(length(u)))*Tp,u); % wyswietlamy u w czasie
