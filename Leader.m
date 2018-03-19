clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'Leader');        % Set your problemname. If you 
                                            % skip this, all files will
                                            % be named "myAcadoProblem"

    DifferentialState x;                    % x position 
    DifferentialState y;                    % y position
    DifferentialState dx;                   % x velocity
    DifferentialState dy;                   % y velocity
    
    Control ux uy;                              % Control inputs
   
     
    %% Diferential Equation
    f = acado.DifferentialEquation();       % Set the differential equation object
   
    f.add(dot(x) == dx);                     % Write down your ODE. 
    f.add(dot(y) == dy);                  
    f.add(dot(dx) == ux);      %
    f.add(dot(dy) == uy);      %
    
    % inputs to the functions
    x0     = acado.MexInput;
    y0     = acado.MexInput;
    dx0    = acado.MexInput;
    dy0    = acado.MexInput;
    rx     = acado.MexInput;
    ry     = acado.MexInput;
    rdx    = acado.MexInput;
    rdy    = acado.MexInput;



    %% Optimal Control Problem
    T = 2; % final time horizon
    N = 10; % itereations
    ocp = acado.OCP(0.0, T, N);         
    
    L = (x-rx)*(x-rx) + (dx-rdx)*(dx-rdx) + (y-ry)*(y-ry) + (dy-rdy)*(dy-rdy);                                        

    ocp.minimizeLagrangeTerm(L);               % Minimize the consumed energy
    
    ocp.subjectTo( f );                     % Optimize with respect to your differential equation
    ocp.subjectTo( 'AT_START', x  ==  x0 ); % initial conditions and constraints
    ocp.subjectTo( 'AT_START', dx ==  dx0);
    ocp.subjectTo( 'AT_START', y  ==  y0 );
    ocp.subjectTo( 'AT_START', dy ==  dy0);
    ocp.subjectTo( -50 <= dx <=  50 );   
    ocp.subjectTo( -50 <= dy <=  50 );   
    ocp.subjectTo( -10 <= ux <= 10 );
    ocp.subjectTo( -10 <= uy <= 10);
    
    %% Optimization Algorithm
    algo =acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm, link it to your OCP
    algo.set( 'KKT_TOLERANCE', 1e-6 );      % Set a custom KKT tolerance
    algo.set('PRINTLEVEL', 'NONE');
    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.
