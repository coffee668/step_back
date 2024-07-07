%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%功能说明：S函数仿真系统的状态方程X(k+1)=A*x(k)+B*u(k)
function [sys,x0,str,ts,simStateCompliance] = ctrl(t,x,u,flag,Pa)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
switch flag
  case 0  %系统进行初始化，调用mdlInitializeSizes函数
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1
    sys=mdlDerivatives(t,x,u);
  case 2  %更新离散化状态变量，调用mdlUpdate函数
    sys=mdlUpdate(t,x,u);
  case 3  % 计算S函数的输出，调用mdlOutputs
    sys=mdlOutputs(t,x,u,Pa);
  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9
    sys=mdlTerminate(t,x,u);
    otherwise   %其他未知情况处理，自定义其他情况
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%0. 系统初始化子函数,定义S-function模块的基本特性，包括采样时间，连续和离散状态的初始化条件，以及sizes数组
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;  %连续量为0
sizes.NumDiscStates  = 0;  %离散状态0个
sizes.NumOutputs     = 1;  %输出1维
sizes.NumInputs      = 5;  %输入5维
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % 至少需要的采样时间1s

sys = simsizes(sizes);

x0  = [];   %设定状态变量初值

str=[];   %str总是设置为空

ts=[0 0];   %表示该模块采样时间继承其前的模块采样时间设置

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simStateCompliance = 'UnknownSimState';
%1.   连续状态的导数，默认为空,计算连续状态变量的导数
function sys=mdlDerivatives(t,x,u)

sys = [];%dx


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%2. 离散状态的更新,更新离散状态、采样时间、主步长等必需条件
function sys=mdlUpdate(t,x,u)

sys = [];   %将计算的结果返回给主函数

%3. 计算S-function的输出
function sys=mdlOutputs(t,x,u,Pa)
x1d=u(1);
dotx1d=u(2);
ddotx1d=u(3);
x1=u(4);
x2=u(5);

k=Pa.k;
k1=Pa.k1;
k2=Pa.k2;
m=Pa.m;
e1=x1d-x1;
e2=dotx1d+k1*e1-x2;

uc=m*e1+m*ddotx1d+m*k1*(dotx1d-x2)+k*x1^3+m*k2*e2;


sys=uc;

%4. 计算下一个采样点的绝对时间，只有当在mdlInitializeSizes中指定了变步长离散采样时间时，才使用该程序
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

%9. 终止函数，执行Simulink终止时所需的任何任务
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
