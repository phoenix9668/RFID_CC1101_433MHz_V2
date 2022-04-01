close all;
clc;
fs = 25;%采样率32kHz
Ts = 1/fs;%采样周期
N = 164;%采样点数168个
m = 0:N-1;%时域采样信号序列编号，也是频域分析频率的序列编号
t = m*Ts;%时域信号横坐标
f = m*fs/N;%频域横坐标

origin=xlsread('origin_data.xlsx','Sheet1','A1:C164');
Xn=origin(1:N,1);
Yn=origin(1:N,2);
Zn=origin(1:N,3);

fir1=[0.3,0.3,0.3];

YRn=conv(Yn,fir1);
N1=length(YRn);
m1 = 0:N1-1;%时域采样信号序列编号，也是频域分析频率的序列编号
t1 = m1*Ts;%时域信号横坐标
f1 = m1*fs/N1;%频域横坐标

N2=length(fir1);
m2 = 0:N2-1;%时域采样信号序列编号，也是频域分析频率的序列编号
t2 = m2*Ts;%时域信号横坐标
f2 = m2*fs/N2;%频域横坐标

Ym = fft(Yn);%对xn做fft
YRm = fft(YRn);%对xn做fft
FIRm = fft(fir1);%对xn做fft

figure;
%做时域图
% subplot(3,1,1);plot(t,X);title('X轴数据');
% subplot(3,1,2);plot(t,Y);title('Y轴数据');
% subplot(3,1,3);plot(t,Z);title('Z轴数据');
%做时域图
subplot(2,3,1);plot(t,Yn);
subplot(2,3,2);plot(t1,YRn);
subplot(2,3,3);plot(t2,fir1);
zoom on; grid on;
title('Time Domain');
xlabel('t(s)');ylabel('amplitude');
%做频域图
subplot(2,3,4);stem(f,abs(Ym),'filled');
subplot(2,3,5);stem(f1,abs(YRm),'filled');
subplot(2,3,6);stem(f2,abs(FIRm),'filled');
zoom on; grid on;
title('Frequency Domain');
xlabel('f(Hz)');ylabel('amplitude');


