classdef HMMProblem < handle
    % A wrapper for the HMM classes to set up runs on the example problem
    % from the original iHMM paper (Beal et al 2003).
    %
    % Jonathan Scholz
    % jkscholz@gatech.edu
    % 11/2/2011
    
    properties(Access=public)
        hmm; % A public hmm type that will be accessible to outside users after run is complete
    end
    
    methods(Access=public)
        function runFixed(obj, nStates, nSamplingIterations, showProgress, collectSamples)
            obj.setUpFixedHMM(nStates);
            obj.hmm.runSampler(nSamplingIterations, collectSamples, showProgress);
            obj.hmm.sortCPT();
            obj.hmm.plot();
        end
        
        function setUpFixedHMM(obj, nStates)
            obj.hmm = HMM();
            incrStepSize = 1; % experimental parameter to make gibbs sampling step size adjustable
            softMaxBeta = 1; % experimental parameter to softmax the likelihood distribution (makes it greedier)
            
            % Set up data for this problem
            [stateSequence, emissionSequence] = HMMProblem.initalizeSequences(nStates);
            
            % get data structures
            stateCPT = ConditionalProbabilityTable();
            emissionCPT = ConditionalProbabilityTable();
            
            % initialize data structures
            HMM.setCPTFromSequence(stateCPT, emissionCPT, stateSequence, emissionSequence, incrStepSize);
            
            % initialize the actual HMM
            obj.hmm.initialize(stateSequence, emissionSequence, stateCPT, emissionCPT, incrStepSize, softMaxBeta);
        end
        
        
        function runHDP(obj, nStates, nSamplingIterations, showProgress, collectSamples)
            obj.setUpHDPHMM(nStates);
            obj.hmm.runSampler(nSamplingIterations, collectSamples, showProgress);
            %obj.hmm.reorderStates();
            obj.hmm.plot();
        end
        
        function setUpHDPHMM(obj, nStates)
            obj.hmm = HDP_HMM();
            incrStepSize = 1; % experimental parameter to make gibbs sampling step size adjustable
            softMaxBeta = 1; % experimental parameter to softmax the likelihood distribution (makes it greedier)
            
            % Set up data for this problem
            [stateSequence, emissionSequence] = HMMProblem.initalizeSequences(nStates);
            
            % get data structures
            stateHDP = HDP();
            emissionHDP = HDP();
            
            % initialize data structures
            HDP_HMM.setHDPFromSequence(stateHDP, emissionHDP, stateSequence, emissionSequence, incrStepSize);
            
            % initialize the actual HMM
            obj.hmm.initialize(stateSequence, emissionSequence, stateHDP, emissionHDP, incrStepSize, softMaxBeta);
        end
    end
    
    methods(Access=public, Static)
        function [stateSequence, emissionSequence] = initalizeSequences(nStates)
            % Sets up the ascending-decending sequence example from Beal et
            % al 2003
            
            % Set a random state sequence
            seqLength = 300;
            stateSequence = setRandomSequence(nStates,seqLength);
            
            % Or pass the perfect sequence
            %stateSequence = repmat([1,2,3,4,5,6,7,8,9,10],1,30);
            
            % Set the Emission sequence
            emissionSequence = repmat([1,2,3,4,5,6,5,4,3,2],1,30);
            
            % Helper: Builds uniformly random trajectory with nPossibleStates
            function seq = setRandomSequence(nPossibleStates, len)
                seq = floor(nPossibleStates*rand(1,len)+1);
            end
        end
    end
end