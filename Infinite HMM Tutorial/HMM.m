classdef HMM < handle
    % Implements a discrete-state, discrete-emission hidden markov model.
    % Supports methods for training the underlying distribution using
    % gibbsd sampling, estimating state trajectory likelihoods given
    % emissions, and emission likehoods using a particle filter.
    %
    % Note that the CPT objects should be derived from the
    % ConditionalProbabilityTable class (e.g. HDP)
    %
    % This implementation roughly follows Beal et al. 2003
    %
    % Jonathan Scholz
    % jkscholz@gatech.edu
    % 11/2/2011
    
    properties(Access=public)
        stateCPT;       % The Conditional probability table on state transitions
        emissionCPT;    % The Conditional probability table on state emissions
        
        E = []; % some vector of observed tokens, which we'll try to model
        S = []; % state indicator variables, one for each token in the sequence X
        
        % Record-related members
        samples = {};   % cell container for storing samples (snapshots of HMM obj)
        burnIn = 10;    % Number of initial samples to discard
        sampFreq = 1;   % frequency of extracting samples after burn-in period
        dispFreq = 1;   % frequency of plotting progress
        ll = [];        % vector of log-likelihoods
        nstates = [];   % counter for how many states were represented at any given time
        sweep = 1;      % sweep counter
        
        %% Experimental:
        incrStepSize; % experimental parameter to make gibbs sampling step size adjustable
        softMaxBeta; % experimental parameter to softmax the likelihood distribution (makes it greedier)
    end
    
    methods(Access=public)
        %% Main interface functions
        function initialize(obj, stateSequence, emissionSequence, stateCPT, emissionCPT, incrStepSize, softMaxBeta)
            obj.S = stateSequence;
            obj.E = emissionSequence;
            obj.stateCPT = stateCPT;
            obj.emissionCPT = emissionCPT;
            obj.incrStepSize = incrStepSize;
            obj.softMaxBeta = softMaxBeta;
            obj.ll = [];
            obj.nstates = [];
            obj.sweep = 1;
        end
        
        function runSampler(obj, numIterations, collectSamples, showProgress)
            % Iterate
            obj.printInfo();
            
            for i = obj.sweep:obj.sweep+numIterations % allows us to resume aborted runs
                obj.sweep = i;
                
                % Resample hidden state sequence
                for t = 1:length(obj.E)
                    obj.S(t) = obj.resampleState(t, true, true, true);
                end
                
                % Resample hyper parameters
                obj.resampleHypers();
                
                % Extract sample
                if collectSamples && i > obj.burnIn && mod(i,obj.sampFreq)==0
                    obj.samples{end+1} = obj.copy();
                end
                
                % Visualize and print info
                if showProgress && mod(i,obj.dispFreq) == 0
                    obj.plot();
                    fprintf('%d: ', i);
                    obj.printInfo();
                end
            end
        end
        
        function [] = printInfo(obj)
            % Update number of states used and the log-likelihood
            obj.nstates(end+1,:) = [obj.sweep; obj.getNumStatesUsed()];
            obj.ll(end+1,:) = [obj.sweep; obj.computeSequenceLogLikelihood()];
            fprintf('----------------------------------------------------\n');
            fprintf('Log-Likelihood: %2.4f\n', obj.ll(end,2));
            fprintf('#states used: %d [ ',obj.nstates(end,2));fprintf('%d ',histc(obj.S, unique(obj.S))); fprintf(']\n');
        end
        
        function plot(obj)
            %obj.sortCPT();
            subplot(2,2,1);
            obj.plotStateSequence();
            subplot(2,2,2);
            %plotEmissionSequence(obj);
            plotLogLikelihood(obj);
            
            subplot(2,2,3);
            obj.plotTransitionMatrix();
            subplot(2,2,4);
            obj.plotEmissionMatrix();
            
            drawnow;
        end
        
        function plotEmissionSequence(obj)
            plot(obj.E, '--rs','MarkerEdgeColor','a','MarkerFaceColor','b','MarkerSize',4);
            set(gca, 'YTick', 0:length(unique(obj.E)));
            title('Emission Sequence');
        end
        
        function plotStateSequence(obj)
            iu = unique(obj.S);
            for i=1:length(obj.S)
                S(i) = find(iu == obj.S(i));
            end
            n_yticks = 5;
            plot(S, '--rs','MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',4);
            %set(gca, 'YTick', floor(length(iu)/n_yticks * (0:n_yticks)));
            title('State Trajectory');
        end
        
        function plotTransitionMatrix(obj)
            imagesc(obj.stateCPT.getCountMatrix());
            title('Transition Matrix');
        end
        
        function plotEmissionMatrix(obj)
            imagesc(obj.emissionCPT.getCountMatrix());
            title('Emission Matrix');
        end
        
        function plotLogLikelihood(obj)
            %plot(obj.ll(:,1), obj.ll(:,2));
            %plot(obj.nstates(:,1), obj.nstates(:,2));
            plotyy(obj.nstates(:,1), obj.nstates(:,2), obj.ll(:,1), obj.ll(:,2));
            h = legend('Number of States', 'Log-likelihood');
            pos = get(h,'position');
            set(h, 'position',[pos(1) pos(2)-0.36 pos(3:4)])
            title('Number of States and overall Log-likelood vs iter')
        end
        
        function nStates = getNumStatesUsed(obj)
            %nStates = max(obj.S); % non-DP version
            nStates = length(unique(obj.S)); % non-DP version
        end
        
        function nStates = getNumStatesRepresented(obj)
            nStates = size(obj.stateCPT.getCountMatrix(),1); % non-DP version
        end
        
        function nTokens = getNumTokens(obj)
            nTokens = max(obj.E);
        end
        
        function sortCPT(obj)
            % Compute permutation vector using the most frequent transition per row
            dps=obj.stateCPT.getCountMatrix();
            P = find(dps(1,:)==max(dps(1,:)), 1 ); % select max of first row
            dps(:,P(end)) = -1; % zero this row so it can't be selected again
            for i=2:length(dps)
                P(i) = find(dps(P(i-1),:)==max(dps(P(i-1),:)), 1 );
                dps(:,P(end)) = -1;
            end
            P = circshift(P,[1 1]); % easiest way to keep from rotating full table every time
            assert(sum(unique(P)==1:10)==size(dps,1));
            
            % Apply permutation to state sequence
            s=obj.S+10;
            p=P+10;
            for i=1:length(s)
                obj.S(i) = find(s(i)==p);
            end
            
            % Apply permutation to LowLevelDPS and LowLevelDPE
            sCounts = obj.stateCPT.getCountMatrix();
            eCounts = obj.emissionCPT.getCountMatrix();
            obj.stateCPT.setCountMatrix(sCounts(P,P));
            obj.emissionCPT.setCountMatrix(eCounts(P,:));
        end
        
        function [meanstateTable, meanemissionTable, meanStateTraj] = computePosterior(obj)
            if ~isempty(obj.samples)
                nSamples = length(obj.samples);
                
                meanstateTable = zeros(10,10);
                meanemissionTable = zeros(10,6);
                meanStateTraj = zeros(length(obj.S),1);
                for i=1:nSamples;
                    meanemissionTable = meanemissionTable + obj.samples{i}.emissionCPT.getCountMatrix();
                    meanstateTable = meanstateTable + obj.samples{i}.stateCPT.getCountMatrix();
                    meanStateTraj = meanStateTraj + obj.samples{i}.S;
                end
                meanemissionTable = meanemissionTable / nSamples;
                meanstateTable = meanstateTable / nSamples;
                meanStateTraj = meanStateTraj / nSamples;
            end
        end
    end
    
    methods(Access=public) % protected
        %% Primary non-static computation functions
        function newState = resampleState(obj, t, usePreviousState, useEmissionSymbol, useNextState)
            % Want: P(S_[t-1] | S_t) * P(E_[t] | S_t) * P(S_[t+1] | S_t)
            %
            % @param t The index to resample
            % @param which variables to condition on
            %   1: usePreviousState
            %   2: useEmissionSymbol
            %   3: useNextState
            
            % get current variables
            currentState = obj.S(t);
            previousState = 1;
            currentEmission = -1;
            nextState = 1;
            
            % Verify that it's okay to use our neighboring states
            usePreviousState = usePreviousState && t ~= 1;
            useNextState  = useNextState && t ~= length(obj.E);
            
            % Set the requested conditioning variables
            if usePreviousState
                previousState = obj.S(t-1);
            end
            if useEmissionSymbol
                currentEmission = obj.E(t);
            end
            if useNextState
                nextState = obj.S(t+1);
            end
            
            %% Decrement counters of the current state and emission
            obj.decrementTransition(previousState, currentState, nextState);
            obj.decrementEmission(currentState, currentEmission);
            
            %% Compute the CPT's for the current count stats
            %sCPT = obj.stateCPT.getCountMatrix();
            %eCPT = obj.emissionCPT.getCountMatrix();
            sCPT = obj.stateCPT.getCPT();
            eCPT = obj.emissionCPT.getCPT();
            
            %% Initialize all likelihoods to uniform
            nStates = size(sCPT,2); %obj.getNumStatesRepresented(); % size(sCPT,2)
            LikelihoodPS = ones(nStates,1);
            LikelihoodE = ones(nStates,1);
            LikelihoodNS = ones(nStates,1);
            
            if usePreviousState
                %hack bc they like different likelihoods for some reason
                LikelihoodPS = HMM.computeRowLikelihood(sCPT,previousState, strcmp(class(obj),'HDP_HMM'));
            end
            
            if useEmissionSymbol
                LikelihoodE = HMM.computeColumnLikelihood(eCPT,currentEmission);
            end
            
            if useNextState
                LikelihoodNS = HMM.computeColumnLikelihood(sCPT,nextState);
            end
            
            %% Compute Overall Likelihood
            try
                Likelihood = LikelihoodPS.*LikelihoodE.*LikelihoodNS;
            catch err
                keyboard
            end
            
            % Alternative to laplace smoothing - if supports don't
            % intersect, just use a uniform
            if sum(Likelihood) == 0
                Likelihood = ones(nStates,1);
            end
            
            %% Draw sample
            %assert(sum(Likelihood) ~= 0);
            % Normalize
            conditionalDistribution = Likelihood/sum(Likelihood);
            
            % Soft-max it to make it greedier
            %conditionalDistribution = HMM.softMax(conditionalDistribution, obj.softMaxBeta);
            
            %resample from conditional distribution
            newState = HMM.sampleFromDistribution(conditionalDistribution);
            
            %% Increment Counters of the chosen state and emission
            obj.incrementTransition(previousState, newState, nextState);
            obj.incrementEmission(newState, currentEmission);
        end
        
        function resampleHypers(obj)
            HMM.setCPTFromSequence(obj.stateCPT, obj.emissionCPT, obj.S, obj.E, obj.incrStepSize)
        end
        
        function logLikelihood = computeSequenceLogLikelihood(obj)
            sCPT = obj.stateCPT.getCPT();% obj.stateCPT.getCountMatrix()
            eCPT = obj.emissionCPT.getCPT(); % obj.emissionCPT.getCountMatrix()
            
            %% Sum of state Log-Likelihoods log(P(S_[t-1],E_[t],S_[t+1]|S_[t])
            % Get likelihood given first state
            LikelihoodE = HMM.computeColumnLikelihood(eCPT,obj.E(1));
            LikelihoodNS = HMM.computeColumnLikelihood(sCPT,obj.S(2));
            logLikelihood = log(LikelihoodE(obj.S(1))) + log(LikelihoodNS(obj.S(1)));
            
            % Loop through body of trajectory
            for t = 2:length(obj.E)-1
                LikelihoodPS = HMM.computeRowLikelihood(sCPT,obj.S(t-1), strcmp(class(obj),'HDP_HMM'));
                LikelihoodE = HMM.computeColumnLikelihood(eCPT,obj.E(t));
                LikelihoodNS = HMM.computeColumnLikelihood(sCPT,obj.S(t+1));
                logLikelihood = logLikelihood + ...
                    log(LikelihoodPS(obj.S(t))) + ...
                    log(LikelihoodE(obj.S(t))) + ...
                    log(LikelihoodNS(obj.S(t)));
            end
            
            % Get likelihood given last state
            LikelihoodE = HMM.computeColumnLikelihood(eCPT,obj.E(end));
            LikelihoodPS = HMM.computeRowLikelihood(sCPT,obj.S(end-1), strcmp(class(obj),'HDP_HMM'));
            logLikelihood = logLikelihood + ...
                log(LikelihoodPS(obj.S(end))) + ...
                log(LikelihoodE(obj.S(end)));
            
        end
        
        %% Non-static helper functions
        function decrementTransition(obj, previousState, currentState, nextState)
            if previousState > 0 && currentState > 0
                obj.stateCPT.decrementCountMatrix(previousState, currentState, obj.incrStepSize);
            end
            if currentState > 0 && nextState > 0
                obj.stateCPT.decrementCountMatrix(currentState, nextState, obj.incrStepSize);
            end
        end
        
        function incrementTransition(obj, previousState, currentState, nextState)
            if previousState > 0 && currentState > 0
                obj.stateCPT.incrementCountMatrix(previousState, currentState, obj.incrStepSize);
            end
            if currentState > 0 && nextState > 0
                obj.stateCPT.incrementCountMatrix(currentState, nextState, obj.incrStepSize);
            end
        end
        
        function decrementEmission(obj, currentState, currentEmission)
            if currentState > 0 && currentEmission > 0
                obj.emissionCPT.decrementCountMatrix(currentState, currentEmission, obj.incrStepSize);
            end
        end
        
        function incrementEmission(obj, currentState, currentEmission)
            if currentState > 0 && currentEmission > 0
                obj.emissionCPT.incrementCountMatrix(currentState, currentEmission, obj.incrStepSize);
            end
        end
        
        function newObj = copy(obj)
            newObj = feval(class(obj));
            proplist = properties(obj);
            for i=1:length(proplist)
                if ~strcmp(proplist{i},'samples')
                    newObj.(proplist{i}) = obj.(proplist{i}); % this is probably broken now that i represent CPT's with a custom class
                end
            end
        end
    end
    
    methods(Access=public, Static) % private
        %% Static likelihood functions
        % Computes a likelihood that conditions on a column entry of the
        % CPT.  I.E. infer backwards in the generative model, from
        % emission or next state to current state.
        function Likelihood = computeColumnLikelihood(CPT, columnIdx)
            % Extract the column corresponding to the columnIdx from the provided CPT
            col = CPT(:,columnIdx);
            
            % Divide the column pointwise by the row sums
            Likelihood = col./sum(CPT,2);
            
            % Remove NaNs
            Likelihood(isnan(Likelihood))=0;
        end
        
        % Computes a likelihood that conditions on a row entry of the
        % CPT.  I.E. infer forwards in the generative model, from
        % previous state to new state or current state to emission.
        %function Likelihood = computeRowLikelihood(CPT, rowIdx)
        function Likelihood = computeRowLikelihood(CPT, rowIdx, isHDP)
            % Extract the row corresponding to the rowIdx from the provided CPT
            row = CPT(rowIdx,:);
            
            % Divide the row pointwise by the column sum
            if isHDP
                Likelihood = [row / sum(row)]';   % correct
            else
                Likelihood = [row./sum(CPT,1)]'; % Wrong, but makes fixed version run better! ... actually this is the full bayesian version, right?
            end
            
            % Remove NaNs
            Likelihood(isnan(Likelihood))=0;
        end
        
        %% Static helpers
        % Generate CPT from a sequence of states and emissions
        function setCPTFromSequence(stateCPT, emissionCPT, S,E,incrStepSize)
            assert(length(S)==length(E));
            numStates = max(S);
            numTokens = max(E);
            stateCPT.setCountMatrix(zeros(numStates,numStates));
            emissionCPT.setCountMatrix(zeros(numStates,numTokens));
            
            emissionCPT.incrementCountMatrix(S(1),E(1), incrStepSize);
            stateCPT.incrementCountMatrix(1,S(1), incrStepSize);
            for t=2:length(S)
                % Update Emission CPT
                emissionCPT.incrementCountMatrix(S(t),E(t), incrStepSize);
                
                % Update Transition CPT
                stateCPT.incrementCountMatrix(S(t-1),S(t), incrStepSize);
            end
        end
        
        function index = sampleFromDistribution(distribution)
            assert(HMM.equals(sum(distribution), 1.0));
            index = sum(rand > cumsum(distribution)) + 1;
        end
        
        function newDistribution = softMax(distribution, beta)
            d=distribution .^ beta;
            newDistribution = d/sum(d);
        end
        
        function b = equals(x,y)
            b = abs(x-y) < 1e-6;
        end
    end
end