close all;
clc;
fs = 25;%采样率32kHz
Ts = 1/fs;%采样周期
N = 340;%采样点数168个
m = 0:N-1;%时域采样信号序列编号，也是频域分析频率的序列编号
t = m*Ts;%时域信号横坐标
f = m*fs/N;%频域横坐标

origin=xlsread('origin_datas28.xlsx','Sheet1','A2:C341');
Xn=origin(1:N,1);
Yn=origin(1:N,2);
Zn=origin(1:N,3);

fir1=[0.3,0.3,0.3];

XRn=conv(Xn,fir1);
Nx=length(XRn);
mx = 0:Nx-1;%时域采样信号序列编号，也是频域分析频率的序列编号
tx = mx*Ts;%时域信号横坐标
fx = mx*fs/Nx;%频域横坐标

YRn=conv(Yn,fir1);
Ny=length(YRn);
my = 0:Ny-1;%时域采样信号序列编号，也是频域分析频率的序列编号
ty = my*Ts;%时域信号横坐标
fy = my*fs/Ny;%频域横坐标

ZRn=conv(Zn,fir1);
Nz=length(ZRn);
mz = 0:Nz-1;%时域采样信号序列编号，也是频域分析频率的序列编号
tz = mz*Ts;%时域信号横坐标
fz = mz*fs/Nz;%频域横坐标

Xm = fft(Xn);%对xn做fft
XRm = fft(XRn);%对xn做fft
Ym = fft(Yn);%对xn做fft
YRm = fft(YRn);%对xn做fft
Zm = fft(Zn);%对xn做fft
ZRm = fft(ZRn);%对xn做fft

%做时域图
figure;
subplot(2,3,1);plot(t,Xn);title('X轴原始数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
subplot(2,3,2);plot(t,Yn);title('Y轴原始数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
subplot(2,3,3);plot(t,Zn);title('Z轴原始数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
subplot(2,3,4);plot(tx,XRn);title('X轴滤波后数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
subplot(2,3,5);plot(ty,YRn);title('Y轴滤波后数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
subplot(2,3,6);plot(tz,ZRn);title('Z轴滤波后数据');zoom on; grid on;xlabel('t(s)');ylabel('amplitude');
%做频域图
figure;
subplot(2,3,1);stem(f,abs(Xm),'filled');title('X轴原始频域');zoom on; grid on;xlabel('f(Hz)');ylabel('amplitude');
subplot(2,3,2);stem(f,abs(Ym),'filled');title('Y轴原始频域');zoom on; grid on;xlabel('f(Hz)');ylabel('amplitude');
subplot(2,3,3);stem(f,abs(Zm),'filled');title('Z轴原始频域');zoom on; grid on;xlabel('f(Hz)');ylabel('amplitude');
subplot(2,3,4);stem(fx,abs(XRm),'filled');title('X轴滤波后频域');zoom on; grid on;xlabel('f(Hz)');ylabel('amplitude');
subplot(2,3,5);stem(fy,abs(YRm),'filled');title('Y轴滤波后频域');zoom on; grid on;xlabel('f(Hz)');ylabel('amplitude');
subplot(2,3,6);stem(fz,abs(ZRm),'filled');title('Z轴滤波后频域');zoom on; grid on;xlabel('f(Hz)');ylabel('amplitude');


