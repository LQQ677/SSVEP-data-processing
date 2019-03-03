function [sys,x0,str,ts] = SSVEPzhaoming(t,x,u,flag,Host,Socket,...
    waitingBegining,waitingTip,PauseMiddle,ssveplength,PauseFeedback,maxtrialonline,SampleRate,Determination,channels,selectchannles,ccaoffset,ccalen,BaseAver,rep,targetp,SSVEPtar)
% H,P,S,
%H,P,S,NumRows,NumCols,NumCustomCmds,CustomFlashCmd,ShowLetterCmd,ResetSpel
%lerCmd,ISI,SimNumTrials,LetterIndices,ShowLetterPeriod,ResetSpellerPeriod,PausePeriod
% see also:
%


global rtBCI





global serialDevice;
% global recvedFrameCnt;



switch flag,
    case 0
        %%%%%%%%%%%%%%%%%%
        % Initialization %
        %%%%%%%%%%%%%%%%%%
        [sys,x0,str,ts]=mdlInitializeSizes;
%       tic;
        
        rtBCI.SampleRate=SampleRate;
        rtBCI.targets=targetp;
        
        
%         iniSer();   % initial serial
        

        rtBCI.waitBegining=round(waitingBegining*SampleRate/1000);
        rtBCI.waitingTip=round(waitingTip*SampleRate/1000);
        rtBCI.PauseMiddle=round(PauseMiddle*SampleRate/1000);
        rtBCI.SSSVEPlength=round(ssveplength*SampleRate/1000);
        rtBCI.PauseFeedback=round(PauseFeedback*SampleRate/1000);
      
        
%         rtBCI.mfp300_without_fb.Status  = NOT_READY;
        
        rtBCI.mfp300_without_fb.decision=Determination;   %   if  need feedback
        
        rtBCI.maxTrialOnline=maxtrialonline; % may be the max trial
        rtBCI.maxTrialcounter=1;
        rtBCI.selectchannles=selectchannles;  %  channels we need

        rtBCI.mfp300_without_fb.sefigure=0;

        
        
        rtBCI.mfp300_without_fb.EvPos = channels;  % the number of all channels
         
        [rtBCI.b , rtBCI.a]=butter(3, [3 40]/(SampleRate/2));
        
        sendSerCom('2-1');
   

        rtBCI.tarPointer=1;
        
        %byzhsj
        %                  for i=1:30\
        
        if rtBCI.mfp300_without_fb.decision==1
            rtBCI.ssvepBuff=zeros(length(selectchannles),ccalen);
            rtBCI.ssvepBuffPointer=1;
            rtBCI.ssvepBuffPointer2=ccalen;
            %             rtBCI.SSVEPtarget=[20 21 22 23 24];
            rtBCI.CCAmatrix=zeros(length(SSVEPtar),4,ccalen);
            for i=1:length(SSVEPtar)
                rtBCI.CCAmatrix(i,:,:)=gensincos(SSVEPtar(i),2,SampleRate,ccalen);
            end
            rtBCI.CCAresult=zeros(size(SSVEPtar));
            rtBCI.dataforCCA=zeros(size(rtBCI.ssvepBuff));
            rtBCI.SSVEPresultold=0;
            
            rtBCI.BaseAver=BaseAver;
            rtBCI.BaseAverCounter=1;
            rtBCI.rep=rep;
            rtBCI.repCounter=1;
            rtBCI.ccaoffset=ccaoffset;
        end
        rtBCI.ccalen=ccalen;
        %         rtBCI.mfp300_without_fb.indices=genOrderbyZhou([0:rtBCI.mfp300_without_fb.NumCustomCmds-1],rtBCI.mfp300_without_fb.maxTrialOnline); %byzhsj
        %         rtBCI.mfp300_without_fb.indicesP300=rtBCI.mfp300_without_fb.indices(rtBCI.mfp300_without_fb.indices<6);  % for skip SSVEP commands in P300 system
        rtBCI.expState=1;
        rtBCI.doOnceID=0;
        rtBCI.timerID=0;
        rtBCI.doItID=0;
        rtBCI.doItID2=0;
        rtBCI.timerID2=0;
    case 3
        
        sys=[10,10,10,10];
        
        
        switch rtBCI.expState,
            case 1
                
                if doOnce(1)
                    sendSerCom('2-1');     % clear tip
                end
                
                if waitAfter(rtBCI.waitBegining,1)
                    moveToStep(2);                 %  wait and next step
                    sys(4)=2;
                end
                
            case 2
                if doOnce(2)
                    if (rtBCI.tarPointer > length( rtBCI.targets) )
                        sendSerCom('2-1');
                    else
                        output = rtBCI.targets(rtBCI.tarPointer);
                        rtBCI.tarPointer=rtBCI.tarPointer+1;
                        sendSerCom(num2str(output+201));
                    end
                end
                
                if waitAfter(rtBCI.waitingTip,2)
                    moveToStep(3);                 %  wait and next step
                    sys(4)=3;
                end
                
            case 3
                if doOnce(3)
                    sendSerCom('2-1');
                end
                
                if waitAfter(rtBCI.PauseMiddle,3)
                    moveToStep(4);
                    sys(4)=4;
                end
                
                
            case 4
                if doOnce(4)
                    sys(1)=50;
                    moveToStep(5);
                    sys(4)=5;
                end
            case 5
                if doOnce(5)
                    sys(2)=207;
                    sendSerCom('207');
                end
                
                
                
                if rtBCI.mfp300_without_fb.decision==1  % if no feedback, just send command...
                    
                    rtBCI.ssvepBuff(:,rtBCI.ssvepBuffPointer)=u( rtBCI.selectchannles );
                    rtBCI.ssvepBuffPointer=rtBCI.ssvepBuffPointer+1;
                    rtBCI.ssvepBuffPointer2=rtBCI.ssvepBuffPointer2-1;
                    if rtBCI.ssvepBuffPointer> rtBCI.ccalen
                        rtBCI.ssvepBuffPointer=1;
                        rtBCI.ssvepBuffPointer2=rtBCI.ccalen;
                    end
                    
                    if waitAfter( rtBCI.ccalen , 5 ) &&  doEvery2( rtBCI.ccaoffset , 5 )
                        rtBCI.BaseAverCounter=rtBCI.BaseAverCounter+1;   % counter the times making result
                        rtBCI.dataforCCA(:,1:rtBCI.ssvepBuffPointer2)=rtBCI.ssvepBuff(:,rtBCI.ssvepBuffPointer:end);
                        rtBCI.dataforCCA(:, rtBCI.ssvepBuffPointer2+1:rtBCI.ssvepBuffPointer2+rtBCI.ssvepBuffPointer-1)=rtBCI.ssvepBuff(:,1:rtBCI.ssvepBuffPointer-1);
                        rtBCI.dataforCCA(isnan(rtBCI.dataforCCA))=100;
                        rtBCI.dataforCCA(isinf(rtBCI.dataforCCA))=100;
                        rtBCI.dataforCCA=filter(rtBCI.b,rtBCI.a, rtBCI.dataforCCA,[],2 );
                        
                        for i4=1:length(rtBCI.CCAresult)
                            rtBCI.CCAresult(i4)=max(cca( rtBCI.dataforCCA,squeeze(rtBCI.CCAmatrix(i4,:,:))));
                        end
                        % do some CCA...
                        
                        
                        [~,t41]=max( rtBCI.CCAresult );  % t41 is the result
                        sys(3)=t41+300;
                        
                        
                        if ( rtBCI.SSVEPresultold == t41 ) || ( rtBCI.rep < 2 )
                            rtBCI.repCounter=rtBCI.repCounter+1;
                            if ( rtBCI.BaseAverCounter > rtBCI.BaseAver ) && ( rtBCI.repCounter > rtBCI.rep )
                                outputResult(t41);
                                sys(2)=t41+300;
                                moveToStep( 7 );
                                 sys(4)=7;
                            end
                        else
                            rtBCI.repCounter=2;
                            rtBCI.SSVEPresultold=t41;
                        end
                    end
                    
                end
                
                
                if waitAfter2( rtBCI.SSSVEPlength , 5 )
                    
                    moveToStep( 6 );
                     sys(4)=6;
                end
                
                
                
                
                
            case 6
                
                if doOnce( 6 )
                    sendSerCom( '208' );
                    sys(2)=208;
                    waitAfter2( 1 , 6 );
                    waitAfter( 1 , 6 );
                    doEvery2(1,6);
                    doEvery(1,6);
                end
                
                if waitAfter( rtBCI.PauseMiddle , 6 )
                    rtBCI.maxTrialcounter = rtBCI.maxTrialcounter + 1;
                    if  rtBCI.maxTrialcounter > rtBCI.maxTrialOnline
                        moveToStep( 9 );
                        sys(4)=9;
                    else
                        moveToStep( 5 );
                        sys(4)=5;
                    end
                end
                
            case 7
                 if doOnce( 7 )
                    sendSerCom( '208' );
                    sys(2)=208;
                 end
                
                if waitAfter( rtBCI.PauseFeedback , 7 )
                    moveToStep( 9 );
                    sys(4)=9;
                end
                
            case 9
                
                if rtBCI.mfp300_without_fb.decision==1
                    rtBCI.ssvepBuffPointer=1;
                    rtBCI.ssvepBuffPointer2=rtBCI.ccalen;
                    rtBCI.BaseAverCounter=1;
                    rtBCI.SSVEPresultold=0;
                    rtBCI.repCounter=1;
                end
                
                rtBCI.maxTrialcounter=1;
                doOnce(9);
                waitAfter2( 1 , 9 );
                waitAfter( 1 , 9 );
                doEvery2(1,9);
                doEvery(1,9);
                
                moveToStep( 4 );
                
                
                
                sys(4)=4;
            otherwise
                
                error(['Unhandled flag = ',num2str(flag)]);
        end
        
        

    case 9
        %%%%%%%%
        % stop %
        %%%%%%%%
        %         if rtBCI.mfp300_without_fb.decision==1&&rtBCI.mfp300_without_fb.realcontrol==1
        %             if( isempty(rtBCI.tss.getRunningBehaviors()) )
        %                 rtBCI.cDefaultBehaviors=rtBCI.tss.getDefaultBehaviors();
        %                 for n=1:size(rtBCI.cDefaultBehaviors,1);
        %                     rtBCI.tss.removeDefaultBehavior(rtBCI.cDefaultBehaviors{n,1})
        %                 end
        %                 rtBCI.tss.addDefaultBehavior('finishtask');
        %                 rtBCI.tss.playDefaultProject();
        %             end
        %         end
%         serialDevice.LED_BLINK_STOP('GID',1);
        sendSerCom('2-1');
        %         sendSerCom('505');%%zhsj
        %         jnet=gUDPclose(rtBCI.mfp300_without_fb.h1);
%         closeSer();
        %         rtBCI = rmfield(rtBCI, 'mfp300_jh_without_fb');
        sys=[];
        % time=rtBCI.udpflashtime;
        
        %         save ('C:\Users\ECUST_BCI\Desktop\flashtime.mat','time');
        
    case { 1, 2, 4}
        %%%%%%%%%%%%%%%%%%%
        % Unhandled flags %
        %%%%%%%%%%%%%%%%%%%
        sys=[];
        
    otherwise
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Unexpected flags (error handling)%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        error(['Unhandled flag = ',num2str(flag)]);
end
end
%==========================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%==========================================================================
%
function [sys,x0,str,ts] = mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = -1;  % dynamically sized
sizes.DirFeedthrough = 1;   % has direct feedthrough
sizes.NumSampleTimes = 1;

sys=simsizes(sizes);
str = [];
x0  = [];
ts  = [-1 0];   % inherited sample time
end
%=================================================================
% %LDA
% function [d2] = LDAtest(samples,V,p1,p2,u1,u2,L1,L2)
% samples=samples';
% m_size = size(samples);
% m = m_size(2);
% inV = inv(V);
% A1 = inV*u1;
% B1 = 0.5*(u1'*A1);
% lgp1 = log(p1);
%
% A2 = inV*u2;
% B2 = 0.5*(u2'*A2);
% lgp2 = log(p2);
% for i=1:m
%     d1 = samples(:,i)'*A1-B1+lgp1;
%     d2 = samples(:,i)'*A2-B2+lgp2;
% end
% end
%
% function varargout = classifybye(b, x)
% x = [x; ones(1,size(x,2))];
%
%
% %% compute mean of predictive distributions
% m = b.w'*x;
%
%
% %% if one output argument return mean only
% if nargout == 1
%     varargout(1) = {m};
% end
%
% %% if two output arguments compute and return variance also
% if nargout == 2
%     s = zeros(1,size(x,2));
%     for i = 1:size(x,2);
%         s(i) = x(:,i)'*b.p*x(:,i) + (1/b.beta);
%     end
%     varargout(1) = {m};
%     varargout(2) = {s};
% end
%
% if nargout > 2
%     fprintf('Too many output arguments!\n');
% end
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %applyn
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% function x = applyn(n, x)
%
% n_channels = size(x,1);
% n_samples  = size(x,2);
% n_trials   = size(x,3);
%
% x = reshape(x,n_channels,n_samples*n_trials);
%
% switch n.method
%
%     case 'minmax'
%         x = x -  repmat(n.min,1,n_samples*n_trials);
%         x = x ./ repmat(n.max-n.min,1,n_samples*n_trials);
%         x = 2*x - ones(n_channels,n_samples*n_trials);
%
%     case 'z-score'
%         x = x -  repmat(n.mean,1,n_samples*n_trials);
%         x = x ./ repmat(n.std,1,n_samples*n_trials);
%
%     otherwise
%         fprintf('unknown normalization method');
%
% end
%
% x = reshape(x,n_channels,n_samples,n_trials);
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %applyw
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% function x = applyw(w, x)
% n_channels = size(x,1);
% n_samples  = size(x,2);
% n_trials   = size(x,3);
%
% %% clip the data
% x = reshape(x,n_channels,n_samples*n_trials);
% l = repmat(w.limit_l',1,n_samples*n_trials);
% h = repmat(w.limit_h',1,n_samples*n_trials);
% i_l = x < l;
% i_h = x > h;
% x(i_l) = l(i_l);
% x(i_h) = h(i_h);
% x = reshape(x,n_channels, n_samples, n_trials);
%
% end


function sendSerCom(c)
global serialDevice;
% toc;
% display(c);
% tic;
return;


c1=c(1);
c23=str2double(c(2:3));
% display(num2str(c23));
if c1=='1' % for each single stimulus
    
    if c23>6
        if c23==7
            %             serialDevice.LED_BLINK_START('LID',5);
            serialDevice.LED_BLINK_START('GID',2);
        elseif c23==8
            %             serialDevice.LED_BLINK_STOP('GID',1);
            %             serialDevice.LED_BLINK_START('GID',1);
            beep;
            serialDevice.LED_BLINK_STOP('GID',2);
            %             serialDevice.LED_BLINK_STOP('LID',5);
        end
    else
        %         if(c23>3)
        %             c23=c23-3;
        %         end\
        % display([num2str(c23) num2str(toc)]);
        % tic;
        serialDevice.SOUND_PLAY('SID',c23);
    end
elseif c1=='2'
    serialDevice.LED_BLINK_STOP('GID',2);
    serialDevice.LED_ON('LID',c23);
    if (c23>0)
        c4=mod((c23-1),3)+1;  %% for hybird
        %     c4=c23;
        playTips(c4);
    end
    serialDevice.LED_OFF('LID',c23);
    
    %     serialDevice.SOUND_PLAY('SID',c23);
    %     serialDevice.SOUND_PLAY('SID',c23);
elseif c1=='6'
    %    serialDevice.SOUND_PLAY('SID',c23);
end
end

function doit=doOnce(id)
global rtBCI
doit=0;
if rtBCI.doOnceID~=id;
    rtBCI.doOnceID=id;
    doit=1;
end
end

function ifdo=doEvery2(time,id)
global rtBCI;
persistent doeverycounter2;
ifdo=0;
if rtBCI.doItID2==id
    doeverycounter2=doeverycounter2+1;
else
    rtBCI.doItID2=id;
    doeverycounter2=2;
end
if doeverycounter2>time
    doeverycounter2=1;
    ifdo=1;
end
end


function ifdo=doEvery(time,id)
global rtBCI;
persistent doeverycounter;
ifdo=0;
if rtBCI.doItID==id
    doeverycounter=doeverycounter+1;
else
    rtBCI.doItID=id;
    doeverycounter=2;
end
if doeverycounter>time
    doeverycounter=1;
    ifdo=1;
end
end

function nextStep()
global rtBCI
rtBCI.expState=1+rtBCI.expState;
end

function moveToStep(s)
global rtBCI
rtBCI.expState=s;
end

function ifTimeOut=waitAfter(len,id)  % for some waiting in exp
global rtBCI
persistent timerCounter
ifTimeOut=0;
if rtBCI.timerID==id
    timerCounter=timerCounter+1;
else
    rtBCI.timerID=id;
    timerCounter=2;     % if len is 1, the function can output 1
    
end
if timerCounter>len   % put these twice for efficient
    ifTimeOut=1;
end
end

function ifTimeOut=waitAfter2(len,id)  % for some waiting in exp
global rtBCI
persistent timerCounter2
ifTimeOut=0;
if rtBCI.timerID2==id
    timerCounter2=timerCounter2+1;
else
    rtBCI.timerID2=id;
    timerCounter2=2;     % if len is 1, the function can output 1
    
end
if timerCounter2>len   % put these twice for efficient
    ifTimeOut=1;
end
end

function outputResult(i)
sendSerCom(num2str(200+i));
end




function iniSer()
% Init serial
global serialDevice;
global soundTipsData;
serialDevice = EEGSerial('COM5');
closeSer();
serialDevice.open();

serialDevice.SOUND_VOL(99);
serialDevice.SOUND_LOAD(1);

serialDevice.LED_SET_FRQ(1, 16);
serialDevice.LED_SET_FRQ(2, 17);
serialDevice.LED_SET_FRQ(3, 18);
serialDevice.LED_SET_FRQ(4, 19);
serialDevice.LED_SET_FRQ(5, 20);
serialDevice.LED_SET_FRQ(6, 21);
serialDevice.LED_SET_FRQ(7, 22);
serialDevice.LED_SET_FRQ(8, 23);
serialDevice.LED_SET_FRQ(9, 24);


% serialDevice.LED_SET_FRQ(1, 20);
% serialDevice.LED_SET_FRQ(2, 24);
% serialDevice.LED_SET_FRQ(3, 28);
% serialDevice.LED_SET_FRQ(4, 27);
% serialDevice.LED_SET_FRQ(5, 22);
% serialDevice.LED_SET_FRQ(6, 26);
% serialDevice.LED_SET_FRQ(7, 21);
% serialDevice.LED_SET_FRQ(8, 25);
% serialDevice.LED_SET_FRQ(9, 29);

soundTipsData = load('tips/soundtip.mat');

end

function closeSer()
global serialDevice;
serialDevice.close();
end




function playTips(index)
global soundTipsData;
%
% if index>0&&index<=6
%     sound(soundTipsData.soundTip(:,index),soundTipsData.FS,soundTipsData.NBITS);
% end
end
%



% function [Wx, Wy, r] = cca(X,Y)
function  r = cca(X,Y)  % just need r, so change the function name.
% CCA calculate canonical correlations
%
% [Wx Wy r] = cca(X,Y) where Wx and Wy contains the canonical correlation
% vectors as columns and r is a vector with corresponding canonical
% correlations. The correlations are sorted in descending order. X and Y
% are matrices where each column is a sample. Hence, X and Y must have
% the same number of columns.
%
% Example: If X is M*K and Y is N*K there are L=MIN(M,N) solutions. Wx is
% then M*L, Wy is N*L and r is L*1.
%
%
% ?? 2000 Magnus Borga, Linköpings universitet

% --- Calculate covariance matrices ---

z = [X;Y];
C = cov(z.');
sx = size(X,1);
sy = size(Y,1);
Cxx = C(1:sx, 1:sx) + 10^(-8)*eye(sx);
Cxy = C(1:sx, sx+1:sx+sy);
Cyx = Cxy';
Cyy = C(sx+1:sx+sy, sx+1:sx+sy) + 10^(-8)*eye(sy);
invCyy = inv(Cyy);

% --- Calcualte Wx and r ---

[Wx,r] = eig(inv(Cxx)*Cxy*invCyy*Cyx); % Basis in X
r = sqrt(real(r));      % Canonical correlations

% --- Sort correlations ---

% V = fliplr(Wx);		% reverse order of eigenvectors
r = flipud(diag(r));	% extract eigenvalues and reverse their order
[r,I]= sort((real(r)));	% sort reversed eigenvalues in ascending order
r = flipud(r);		% restore sorted eigenvalues into descending order
% for j = 1:length(I)
%     Wx(:,j) = V(:,I(j));  % sort reversed eigenvectors in ascending order
% end
% Wx = fliplr(Wx);	% restore sorted eigenvectors into descending order
%
% % --- Calcualte Wy  ---
%
% Wy = invCyy*Cyx*Wx;     % Basis in Y
% Wy = Wy./repmat(sqrt(sum(abs(Wy).^2)),sy,1); % Normalize Wy
end



function x=gensincos(hz,n,t,l)
% general sin and cos
% hz is the frequency
% n is the n times harmonic will make
% t is the sample rate of data
% l is the length of data
tt=linspace(0,l/t,l);
x=zeros(2*n,l);
for i=0:n-1
    ii=i+1;
    x(1+i*2,:)=sin(2*pi*tt*hz*ii);
    x(2+i*2,:)=cos(2*pi*tt*hz*ii);
    
end
end
