%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [sys,x0,str,ts,simStateCompliance] = plant(t,x,u,flag,Pa)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
switch flag
  case 0  %ϵͳ���г�ʼ��������mdlInitializeSizes����
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1
    sys=mdlDerivatives(t,x,u,Pa);
  case 2  %������ɢ��״̬����������mdlUpdate����
    sys=mdlUpdate(t,x,u);
  case 3  % ����S���������������mdlOutputs
    sys=mdlOutputs(t,x,u);
  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9
    sys=mdlTerminate(t,x,u);
    otherwise   %����δ֪��������Զ����������
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%0. ϵͳ��ʼ���Ӻ���,����S-functionģ��Ļ������ԣ���������ʱ�䣬��������ɢ״̬�ĳ�ʼ���������Լ�sizes����
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 2;  %������Ϊ0
sizes.NumDiscStates  = 0;  %��ɢ״̬3�����ֱ���x-xref��y-yref��yaw-yawref
sizes.NumOutputs     = 2;  %���2ά���ֱ���ǰ��ת�Ǻͳ���
sizes.NumInputs      = 1;  %����6ά���ֱ���x,y,yaw,xref,yref,yawref
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % ������Ҫ�Ĳ���ʱ��1s

sys = simsizes(sizes);

x0  = [0;0];   %�趨״̬������ֵ

str=[];   %str��������Ϊ��

ts=[0 0];   %��ʾ��ģ�����ʱ��̳���ǰ��ģ�����ʱ������

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simStateCompliance = 'UnknownSimState';
%1.   ����״̬�ĵ�����Ĭ��Ϊ��,��������״̬�����ĵ���
function sys=mdlDerivatives(t,x,u,Pa)

m=Pa.m;
k=Pa.k;

x1=x(1);
x2=x(2);

x1dt=x2;
x2dt=-k/m*x1^3+1/m*u;


sys = [x1dt;x2dt];%dx


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%2. ��ɢ״̬�ĸ���,������ɢ״̬������ʱ�䡢�������ȱ�������
function sys=mdlUpdate(t,x,u)

sys = [];   %������Ľ�����ظ�������

%3. ����S-function�����
function sys=mdlOutputs(t,x,u)

sys=x;

%4. ������һ��������ľ���ʱ�䣬ֻ�е���mdlInitializeSizes��ָ���˱䲽����ɢ����ʱ��ʱ����ʹ�øó���
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

%9. ��ֹ������ִ��Simulink��ֹʱ������κ�����
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate

