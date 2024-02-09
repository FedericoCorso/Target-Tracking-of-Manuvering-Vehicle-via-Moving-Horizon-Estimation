function [xstar,fxstar,k,exitflag,xsequence] = myfminunc(fun,x0,myoptions)
% MYFMINUNC Attempts to solve the problem:
%                   min f(x)            
% and, if successful, returns a local minimizer xstar and the related local
% optimum fxstar=f(xstar). The solver employs a quasi-Newton optimization
% scheme and back-tracking line search with Armijo condition.
%
%   INPUTS:
%           fun         =   cost function
%           x0          =   initial guess for the optimization variables
%           gradfxk     =   gradient of the cost function with respect to
%                           x, evaluated at xk
%           myoptions   =   optimization options prepared with myoptimset
%                           command
%
%   OUTPUTS:
%           xstar       =   exit value, either a local minimizer or the
%                           value of x at the last iterate
%           fxstar      =   cost function evaluated at xstar
%           niter       =   number of employed iterations 
%           exitflag    =   termination condition:
%                           -1: max. number of iterations reached
%                            1: local minimum possible, gradient condition
%                            2: local minimum possible, step size condition
%                            3: local minimum possible, cost decrease condition
%           xsequence   =   sequence of iterations {xk} (only if option
%                           xsequence is set to 'on')

%% Initialization
n               =   length(x0);
k               =   0;
deltaxk_rel     =   1; % initialization
deltaf_rel     	=   1; % initialization 
if ~isempty(myoptions.outputfcn)
    outputfun   =   myoptions.outputfcn;
end
if strcmp(myoptions.display,'Iter')
    fprintf('Iteration       NormGrad          Cost      Rel. cost         Rel. x      Line-search\r')
end

xsequence           =   [];
if strcmp(myoptions.xsequence,'on')
    xsequence       =   [xsequence, x0];
end

%% Iterations
if strcmp(myoptions.Hessmethod,'GN')    % Gauss-Newton method
    funF         	=   myoptions.GN_funF;
    xk              =   x0;
    [Fxk,gradFxk]  	=   mygradient(funF,x0,myoptions.gradmethod,myoptions.graddx);
    fxk            	=   Fxk'*Fxk;
    gradfxk        	=   2*gradFxk*Fxk;
    Hk             	=   2*(gradFxk*gradFxk')+myoptions.GN_sigma*eye(n);
    pk            	=   -Hk\gradfxk;
    
    if strcmp(myoptions.display,'Iter')
        fprintf('%9.0f    %7.5e   %6.5e    %6.5e    %6.5e             %4.0f\r',...
            k,norm(gradfxk'*pk),fxk,deltaf_rel,deltaxk_rel,0)
    end
    if ~isempty(myoptions.outputfcn)
        outputfun(xk);
    end
    while abs(gradfxk'*pk) > myoptions.tolgrad...
            && k < myoptions.nitermax...
            && deltaxk_rel > myoptions.tolx...
            && deltaf_rel > myoptions.tolfun
        k                       =   k+1;
        [xkp1,fxkp1,niter_LS]	=  	linesearch(fun,fxk,gradfxk,xk,pk,...
                                    myoptions.ls_tkmax,myoptions.ls_beta,...
                                    myoptions.ls_c,myoptions.ls_nitermax);
        deltaxk_rel             =   norm(xkp1-xk)/max(eps,norm(xk));
        deltaf_rel              =   abs(fxkp1-fxk)/max(eps,abs(fxk));
        % set the new point as current starting point
        xk                      =   xkp1;
        fxk                     =   fxkp1;
        % compute the gradient at kth value and the cost function
        [Fxk,gradFxk]           =   mygradient(funF,xk,...
                                    myoptions.gradmethod,myoptions.graddx);
        gradfxk                 =   2*gradFxk*Fxk;
        Hk                      =   2*(gradFxk*gradFxk')+myoptions.GN_sigma*eye(n);
        % compute the new search direction
        pk                      =   -Hk\gradfxk;

        if strcmp(myoptions.display,'Iter')
            fprintf('%9.0f    %7.5e   %6.5e    %6.5e    %6.5e             %4.0f\r',...
            k,norm(gradfxk'*pk),fxk,deltaf_rel,deltaxk_rel,niter_LS)
        end
        if ~isempty(myoptions.outputfcn)
            outputfun(xk);
        end
        if strcmp(myoptions.xsequence,'on')
            xsequence       =   [xsequence, xk];
        end
    end
elseif strcmp(myoptions.Hessmethod,'Exact')    % Exact Newton method
    xk                  =   x0;
    [fxk,gradfxk,Hk]    =   fun(x0);
    pk                  =   -Hk\gradfxk;
    
    if strcmp(myoptions.display,'Iter')
        fprintf('%9.0f    %7.5e   %6.5e    %6.5e    %6.5e             %4.0f\r',...
            k,norm(gradfxk'*pk),fxk,deltaf_rel,deltaxk_rel,0)
    end
    if ~isempty(myoptions.outputfcn)
        outputfun(xk);
    end
    while abs(gradfxk'*pk) > myoptions.tolgrad...
            && k < myoptions.nitermax...
            && deltaxk_rel > myoptions.tolx...
            && deltaf_rel > myoptions.tolfun
        k                       =   k+1;
        [xkp1,fxkp1,niter_LS]	=  	linesearch(fun,fxk,gradfxk,xk,pk,...
                                    myoptions.ls_tkmax,myoptions.ls_beta,...
                                    myoptions.ls_c,myoptions.ls_nitermax);
        deltaxk_rel             =   norm(xkp1-xk)/max(eps,norm(xk));
        deltaf_rel              =   abs(fxkp1-fxk)/max(eps,abs(fxk));
        xk                      =   xkp1;
        [fxk,gradfxk,Hk]        =   fun(xk);
        pk                      =   -Hk\gradfxk;

        if strcmp(myoptions.display,'Iter')
            fprintf('%9.0f    %7.5e   %6.5e    %6.5e    %6.5e             %4.0f\r',...
            k,norm(gradfxk'*pk),fxk,deltaf_rel,deltaxk_rel,niter_LS)
        end
        if ~isempty(myoptions.outputfcn)
            outputfun(xk);
        end
        if strcmp(myoptions.xsequence,'on')
            xsequence       =   [xsequence, xk];
        end
    end
elseif strcmp(myoptions.Hessmethod,'BFGS')    % BFGS method
    xk              =   x0;
    Hk              =   1e-4*eye(n);
    [fxk,gradfxk]  	=   mygradient(fun,x0,myoptions.gradmethod,myoptions.graddx);
    pk            	=   -Hk\gradfxk;
    
    if strcmp(myoptions.display,'Iter')
        fprintf('%9.0f    %7.5e   %6.5e    %6.5e    %6.5e             %4.0f\r',...
            k,norm(gradfxk'*pk),fxk,deltaf_rel,deltaxk_rel,0)
    end
    if ~isempty(myoptions.outputfcn)
        outputfun(xk);
    end
    while abs(gradfxk'*pk) > myoptions.tolgrad...
            && k < myoptions.nitermax...
            && deltaxk_rel > myoptions.tolx...
            && deltaf_rel > myoptions.tolfun
        k                       =   k+1;
        [xkp1,fxkp1,niter_LS]	=  	linesearch(fun,fxk,gradfxk,xk,pk,...
                                    myoptions.ls_tkmax,myoptions.ls_beta,...
                                    myoptions.ls_c,myoptions.ls_nitermax);
        deltaxk_rel             =   norm(xkp1-xk)/max(eps,norm(xk));
        deltaf_rel              =   abs(fxkp1-fxk)/max(eps,abs(fxk));
        % Update Hessian estimate with BFGS rule
        [~,gradfxkp1]           =   mygradient(fun,xkp1,...
                                    myoptions.gradmethod,myoptions.graddx);
        y                       =   gradfxkp1-gradfxk;
        s                       =   xkp1-xk;
        if y'*s<= myoptions.BFGS_gamma*(s'*Hk*s)
            y   =   y+(myoptions.BFGS_gamma*s'*Hk*s-s'*y)/(s'*Hk*s-s'*y)*(Hk*s-y);
        end
        Hk                      =   Hk-(Hk*(s*s')*Hk)/(s'*Hk*s)+(y*y')/(s'*y);
        xk                      =   xkp1;
        fxk                     =   fxkp1;
        gradfxk                 =   gradfxkp1;
        pk                      =   -Hk\gradfxk;

        if strcmp(myoptions.display,'Iter')
            fprintf('%9.0f    %7.5e   %6.5e    %6.5e    %6.5e             %4.0f\r',...
            k,norm(gradfxk'*pk),fxk,deltaf_rel,deltaxk_rel,niter_LS)
        end
        if ~isempty(myoptions.outputfcn)
            outputfun(xk);
        end
        if strcmp(myoptions.xsequence,'on')
            xsequence       =   [xsequence, xk];
        end
    end
elseif strcmp(myoptions.Hessmethod,'SD')    % Steepest Descent method
    xk              =   x0;
    [fxk,gradfxk]  	=   mygradient(fun,x0,myoptions.gradmethod,myoptions.graddx);
    pk            	=   -gradfxk;
    
    if strcmp(myoptions.display,'Iter')
        fprintf('%9.0f    %7.5e   %6.5e    %6.5e    %6.5e             %4.0f\r',...
            k,norm(gradfxk'*pk),fxk,deltaf_rel,deltaxk_rel,0)
    end
    if ~isempty(myoptions.outputfcn)
        outputfun(xk);
    end
    while abs(gradfxk'*pk) > myoptions.tolgrad...
            && k < myoptions.nitermax...
            && deltaxk_rel > myoptions.tolx...
            && deltaf_rel > myoptions.tolfun
        k                       =   k+1;
        [xkp1,fxkp1,niter_LS]	=  	linesearch(fun,fxk,gradfxk,xk,pk,...
                                    myoptions.ls_tkmax,myoptions.ls_beta,...
                                    myoptions.ls_c,myoptions.ls_nitermax);
        deltaxk_rel             =   norm(xkp1-xk)/max(eps,norm(xk));
        deltaf_rel              =   abs(fxkp1-fxk)/max(eps,abs(fxk));
        xk                      =   xkp1;
        [fxk,gradfxk]           =   mygradient(fun,xk,...
                                    myoptions.gradmethod,myoptions.graddx);
        pk                      =   -gradfxk;

        if strcmp(myoptions.display,'Iter')
            fprintf('%9.0f    %7.5e   %6.5e    %6.5e    %6.5e             %4.0f\r',...
            k,norm(gradfxk'*pk),fxk,deltaf_rel,deltaxk_rel,niter_LS)
        end
        if ~isempty(myoptions.outputfcn)
            outputfun(xk);
        end
        if strcmp(myoptions.xsequence,'on')
            xsequence       =   [xsequence, xk]; % save the sequence of outputs of the optim method
        end
    end
end

%% Termination
xstar   =   xk;
fxstar  =   fxk;
if norm(gradfxk'*pk) <= myoptions.tolgrad
    exitflag    =   1;
    if strcmp(myoptions.display,'Iter')
        fprintf('Local minimum possible, directional derivative smaller than tolerance\r')
    end
elseif k >= myoptions.nitermax
    exitflag    =   -1;
    if strcmp(myoptions.display,'Iter')
        fprintf('Maximum number of iterations reached\r')
    end
elseif deltaxk_rel <= myoptions.tolx
    exitflag    =   2;
    if strcmp(myoptions.display,'Iter')
        fprintf('Local minimum possible, relative step size smaller than tolerance\r')
    end
elseif deltaf_rel <= myoptions.tolfun
    exitflag    =   3;
    if strcmp(myoptions.display,'Iter')
        fprintf('Local minimum possible, relative cost decrease smaller than tolerance\r')
    end
end
    
    
    