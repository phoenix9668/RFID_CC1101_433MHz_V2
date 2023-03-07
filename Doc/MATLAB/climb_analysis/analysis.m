close all;
clc;
fs = 25;%采样率32kHz
Ts = 1/fs;%采样周期
N = 150;%采样点数168个
m = 0:N-1;%时域采样信号序列编号，也是频域分析频率的序列编号
t = m*Ts;%时域信号横坐标
f = m*fs/N;%频域横坐标

origin=xlsread('20230109_1024_.xlsx','Sheet1','A2:D151');
Xn=origin(1:N,2);
Yn=origin(1:N,3);
Zn=origin(1:N,4);
Tn=origin(1:N,1);

Xd = diff(Xn)-2000;
Nd = 0:length(Xd)-1;
Yd = diff(Yn)-2000;
Zd = diff(Zn)-2000;

figure;
plot(m,Xn,'c-*');hold on;plot(m,Tn,'b-');hold on;plot(Nd,Xd,'m-x');
title('X轴原始数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
figure;
plot(m,Yn,'k-*');hold on;plot(m,Tn,'b-');hold on;plot(Nd,Yd,'r-x');
title('Y轴原始数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
figure;
plot(m,Zn,'g-*');hold on;plot(m,Tn,'b-');hold on;plot(Nd,Zd,'b-x');
title('Z轴原始数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');


fir1=[0.33,0.33,0.33];

XRn=conv(Xn,fir1);
Nx=length(XRn);
mx = 0:Nx-1;%时域采样信号序列编号，也是频域分析频率的序列编号
% tx = mx*Ts;%时域信号横坐标
% fx = mx*fs/Nx;%频域横坐标

YRn=conv(Yn,fir1);
Ny=length(YRn);
my = 0:Ny-1;%时域采样信号序列编号，也是频域分析频率的序列编号
% ty = my*Ts;%时域信号横坐标
% fy = my*fs/Ny;%频域横坐标

ZRn=conv(Zn,fir1);
Nz=length(ZRn);
mz = 0:Nz-1;%时域采样信号序列编号，也是频域分析频率的序列编号
% tz = mz*Ts;%时域信号横坐标
% fz = mz*fs/Nz;%频域横坐标

Xd = diff(XRn)-2000;
Nd = 0:length(Xd)-1;
Yd = diff(YRn)-2000;
Zd = diff(ZRn)-2000;

Xm = fft(Xn);%对xn做fft
XRm = fft(XRn);%对xn做fft
Ym = fft(Yn);%对xn做fft
YRm = fft(YRn);%对xn做fft
Zm = fft(Zn);%对xn做fft
ZRm = fft(ZRn);%对xn做fft

%做时域图
figure;
plot(m,Xn,'c-*');hold on;plot(m,Tn,'b-');hold on;plot(Nd,Xd,'m-x');
title('X轴滤波数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
figure;
plot(m,Yn,'k-*');hold on;plot(m,Tn,'b-');hold on;plot(Nd,Yd,'r-x');
title('Y轴滤波数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
figure;
plot(m,Zn,'g-*');hold on;plot(m,Tn,'b-');hold on;plot(Nd,Zd,'b-x');
title('Z轴滤波数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');


% subplot(2,3,1);plot(t,Xn);title('X轴原始数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
% subplot(2,3,2);plot(t,Yn);title('Y轴原始数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
% subplot(2,3,3);plot(t,Zn);title('Z轴原始数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
% subplot(2,3,4);plot(tx,XRn);title('X轴滤波后数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
% subplot(2,3,5);plot(ty,YRn);title('Y轴滤波后数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
% subplot(2,3,6);plot(tz,ZRn);title('Z轴滤波后数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
%做频域图
figure;
subplot(2,3,1);stem(f,abs(Xm),'filled');title('X轴原始频域');zoom on; grid on;xlabel('f(Hz)');ylabel('amplitude');
subplot(2,3,2);stem(f,abs(Ym),'filled');title('Y轴原始频域');zoom on; grid on;xlabel('f(Hz)');ylabel('amplitude');
subplot(2,3,3);stem(f,abs(Zm),'filled');title('Z轴原始频域');zoom on; grid on;xlabel('f(Hz)');ylabel('amplitude');
% subplot(2,3,4);stem(fx,abs(XRm),'filled');title('X轴滤波后频域');zoom on; grid on;xlabel('f(Hz)');ylabel('amplitude');
% subplot(2,3,5);stem(fy,abs(YRm),'filled');title('Y轴滤波后频域');zoom on; grid on;xlabel('f(Hz)');ylabel('amplitude');
% subplot(2,3,6);stem(fz,abs(ZRm),'filled');title('Z轴滤波后频域');zoom on; grid on;xlabel('f(Hz)');ylabel('amplitude');


