%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [sys,x0,str,ts,simStateCompliance] = plant(t,x,u,flag,Pa)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
switch flag
  case 0  %系统进行初始化，调用mdlInitializeSizes函数
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1
    sys=mdlDerivatives(t,x,u,Pa);
  case 2  %更新离散化状态变量，调用mdlUpdate函数
    sys=mdlUpdate(t,x,u);
  case 3  % 计算S函数的输出，调用mdlOutputs
    sys=mdlOutputs(t,x,u);
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
sizes.NumContStates  = 2;  %连续量为0
sizes.NumDiscStates  = 0;  %离散状态3个，分别是x-xref，y-yref，yaw-yawref
sizes.NumOutputs     = 2;  %输出2维，分别是前轮转角和车速
sizes.NumInputs      = 1;  %输入6维，分别是x,y,yaw,xref,yref,yawref
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % 至少需要的采样时间1s

sys = simsizes(sizes);

x0  = [0;0];   %设定状态变量初值

str=[];   %str总是设置为空

ts=[0 0];   %表示该模块采样时间继承其前的模块采样时间设置

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simStateCompliance = 'UnknownSimState';
%1.   连续状态的导数，默认为空,计算连续状态变量的导数
function sys=mdlDerivatives(t,x,u,Pa)

m=Pa.m;
k=Pa.k;

x1=x(1);
x2=x(2);

x1dt=x2;
x2dt=-k/m*x1^3+1/m*u;


sys = [x1dt;x2dt];%dx


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%2. 离散状态的更新,更新离散状态、采样时间、主步长等必需条件
function sys=mdlUpdate(t,x,u)

sys = [];   %将计算的结果返回给主函数

%3. 计算S-function的输出
function sys=mdlOutputs(t,x,u)

sys=x;

%4. 计算下一个采样点的绝对时间，只有当在mdlInitializeSizes中指定了变步长离散采样时间时，才使用该程序
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

%9. 终止函数，执行Simulink终止时所需的任何任务
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate

