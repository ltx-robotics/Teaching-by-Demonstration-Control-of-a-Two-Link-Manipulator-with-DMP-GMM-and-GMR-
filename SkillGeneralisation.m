classdef SkillGeneralisation
    % This SkillGeneralisation class implements the Dynamic Movement Primitives (DMPs).
    % It includes methods for preprocessing demonstration trajectories, performing nonlinear fitting,
    % and reconstructing trajectories. Some functions in this class are incomplete
    % and need to be implemented based on the knowledge from the Robot Learning and Teleoperation module.
    properties
        % Parameters for the DMPs
        numStates double            % Number of activation functions (i.e., number of RBF in the forcing term or K of a GMM )
        numVar int8                 % Number of variables dim([x,s1,s2]) (decay term and perturbing force)
        numVarPos int8              % Dimension of spatial variables dim([s1,s2]) 
        numData int16               % Number of time stamps of a trajectory 
        numDemos int16              % Number of demonstrations 
        beta double                 % Stiffness gain (β)
        alpha double                % Damping gain (α)
        dt double                   % Duration of time step (τ)
        L double                    % Feedback term
        xIn = []                    % State of the canonical system
        dataTraining = []           % Forcing term for training
        startPos double             % Start position of the trajectory
        endPos double               % End position of the trajectory
    end

    methods
        function obj = SkillGeneralisation(dim, states, beta, alpha, dt)
            % Construct an instance of this class
            obj.numStates = states;                 % Number of activation functions 
            obj.numVar = dim;                       % Number of variables
            obj.numVarPos = obj.numVar-1;           % Dimension of spatial variables
            obj.beta = beta;                        % Stiffness gain
            obj.alpha = alpha;                      % Damping gain (with ideal underdamped damping ratio) 
            obj.dt = dt;                            % Duration of time step
            obj.L = [eye(obj.numVarPos)*beta, eye(obj.numVarPos)*alpha]; % Feedback term
        end

        %% Generate time Stamps
        function obj = canonicalSystemInitialisation(obj,decayFactor,numData)
            obj.xIn(1) = 1;
            for t = 2:numData
            	obj.xIn(t) = obj.xIn(t-1) - decayFactor * obj.xIn(t-1) * obj.dt;       % Update of decay term (dx/dt = -ax, τx'=-ax)
            end
            obj.numData = numData;
        end

        %% Compute imaginary force from trajectory
        function obj = trajectort2Forcing(obj, demos)
            obj.numDemos = length(demos);
            numTimeSteps = length(obj.xIn);
            obj.endPos = demos{1}.pos(:,end);
            obj.startPos = demos{1}.pos(:,1);
            for n = 1:obj.numDemos
                pos = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),numTimeSteps));     % Resampling Positions
                [traj(n).pos, traj(n).vel, traj(n).acc] = obj.generateHighOrderTerms(pos);
                oneDemoForceWithTimeStamps = [obj.xIn; (traj(n).acc - (repmat(obj.endPos,1,numTimeSteps)-traj(n).pos)*obj.beta + traj(n).vel*obj.alpha) ./ repmat(obj.xIn,obj.numVarPos,1)];
                obj.dataTraining = [obj.dataTraining, oneDemoForceWithTimeStamps];
            end
        end
        
        %% Compute velocity and acceleration from position trajectory 
        function [pos, vel, acc] = generateHighOrderTerms(obj, pos)
            vel = gradient(pos) / obj.dt;             % Compute Velocity
            acc = gradient(vel) / obj.dt;             % Compute Acceleration
        end

        %% Ordinary DMP with locally weighed locally WLS
        function fOut = fittingWithLocallyWLS(obj)
            % -------------------Add your code here --------------------    👈👈👈
            sTrain = obj.dataTraining(1,:);
            fTrain = obj.dataTraining(2:end,:);

            K = obj.numStates;
            D = obj.numVarPos;
            T = obj.numData;

            % Centres of Gaussian basis functions along the phase variable
            centres = linspace(max(sTrain), min(sTrain), K);

            % Widths of basis functions
            if K > 1
                spacing = abs(centres(2) - centres(1));
            else
                spacing = max(sTrain) - min(sTrain);
            end

            if spacing < eps
                spacing = 1;
            end

            sigma = spacing * 1.5;
            sigma2 = sigma^2;

            theta = zeros(D, K, 2);
            X = [ones(length(sTrain),1), sTrain(:)];

            for k = 1:K
                psi = exp(-0.5 * ((sTrain - centres(k)).^2) / sigma2);
                sqrtPsi = sqrt(psi(:));

                Xw = X .* sqrtPsi;

                for d = 1:D
                    y = fTrain(d,:)';
                    yw = y .* sqrtPsi;

                    A = Xw' * Xw + 1e-8 * eye(2);
                    b = Xw' * yw;

                    coeff = A \ b;

                    theta(d,k,1) = coeff(1);
                    theta(d,k,2) = coeff(2);
                end
            end

            fOut = zeros(D,T);

            for t = 1:T
                s = obj.xIn(t);

                h = exp(-0.5 * ((s - centres).^2) / sigma2);
                h = h ./ (sum(h) + realmin);

                for d = 1:D
                    localPred = squeeze(theta(d,:,1)) + squeeze(theta(d,:,2)) * s;
                    fOut(d,t) = sum(h(:) .* localPred(:));
                end
            end
            % ----------------------------------------------------------    👈👈👈
        end

        %% Optimised DMP with GMM and GMR   
        function fOut = fittingWithGMR(obj)
            % -------------------Add your code here --------------------    👈👈👈
            gmmData = obj.dataTraining';

            gmmodel = MixtureGaussians(gmmData, obj.numStates);

            gmmodel = gmmodel.gmmFit(gmmData, 200, 1e-6, false);

            mask = zeros(1, obj.numVar);
            mask(1) = 1;

            gmmodel = gmmodel.defineQueryDim(mask);

            querys = obj.xIn';

            gmmodel = gmmodel.gaussianMixtureRegression(querys);

            % GMM(R) Plots, debugging use
            if true
                figure;
                gmmodel.plotGmmAndData(gmmData);

                figure;
                plot(gmmodel.logLikelihood,'-r','LineWidth',2);
                title('Log-likelihood during training');
                xlabel('number of iterations')
                ylabel('Log-likelihood')

                figure;
                scatter3(gmmData(:,1), gmmData(:,2), gmmData(:,3), '.b');
                hold on;
                scatter3(gmmodel.regressedTraj(:,1), gmmodel.regressedTraj(:,2), gmmodel.regressedTraj(:,3), '.g');
                hold off
            end

            fOut = gmmodel.regressedTraj(:, gmmodel.outDim)';
            % ----------------------------------------------------------    👈👈👈
        end

        %% Generalise imaginary force back to trajectory
        function trajOut = forcing2Trajectory(obj, forcingTraj)
            x = obj.startPos;
            xTarget = obj.endPos;
            dx = zeros(obj.numVarPos,1);
            for t = 1:obj.numData
            % -------------------Add your code here --------------------    👈👈👈   
            ddx = obj.beta * (xTarget - x) ...
                - obj.alpha * dx ...
                + obj.xIn(t) * forcingTraj(:,t);        % Regenerate acceleration

            dx = dx + ddx * obj.dt;                     % Regenerate velocity

            x = x + dx * obj.dt;                        % Regenerate position

            rData(:,t) = x;
            % ---------------------------------------------------------- 
            end
            trajOut = rData;
        end
    end

    methods (Static)              
        % ----- Add your functions here to help with your evaluation ------
        % -----------------------------------------------------------------
    end
end