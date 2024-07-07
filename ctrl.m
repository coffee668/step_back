%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵����S��������ϵͳ��״̬����X(k+1)=A*x(k)+B*u(k)
function [sys,x0,str,ts,simStateCompliance] = ctrl(t,x,u,flag,Pa)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
switch flag
  case 0  %ϵͳ���г�ʼ��������mdlInitializeSizes����
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1
    sys=mdlDerivatives(t,x,u);
  case 2  %������ɢ��״̬����������mdlUpdate����
    sys=mdlUpdate(t,x,u);
  case 3  % ����S���������������mdlOutputs
    sys=mdlOutputs(t,x,u,Pa);
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
sizes.NumContStates  = 0;  %������Ϊ0
sizes.NumDiscStates  = 0;  %��ɢ״̬0��
sizes.NumOutputs     = 1;  %���1ά
sizes.NumInputs      = 5;  %����5ά
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % ������Ҫ�Ĳ���ʱ��1s

sys = simsizes(sizes);

x0  = [];   %�趨״̬������ֵ

str=[];   %str��������Ϊ��

ts=[0 0];   %��ʾ��ģ�����ʱ��̳���ǰ��ģ�����ʱ������

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simStateCompliance = 'UnknownSimState';
%1.   ����״̬�ĵ�����Ĭ��Ϊ��,��������״̬�����ĵ���
function sys=mdlDerivatives(t,x,u)

sys = [];%dx


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%2. ��ɢ״̬�ĸ���,������ɢ״̬������ʱ�䡢�������ȱ�������
function sys=mdlUpdate(t,x,u)

sys = [];   %������Ľ�����ظ�������

%3. ����S-function�����
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

%4. ������һ��������ľ���ʱ�䣬ֻ�е���mdlInitializeSizes��ָ���˱䲽����ɢ����ʱ��ʱ����ʹ�øó���
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

%9. ��ֹ������ִ��Simulink��ֹʱ������κ�����
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
