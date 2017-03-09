classdef HDP_HMM < HMM
    % An extended version of the regular HMM which uses a hierarchical
    % dirichlet process as the primary probability object rather than a
    % convensional CPT.  The only difference this imparts on the HMM side
    % is the need to decrement and increment the HDP's oracle vector when
    % resampling the hidden state trajectory
    %
    % Jonathan Scholz
    % jkscholz@gatech.edu
    % 11/2/2011
    
    properties(Access=public) %protected
        lastStateOracleIdx = 0; % temp variable for the state oracle counter
        lastEmissionOracleIdx = 0; % temp variable for the emission oracle counter
    end
    
    methods(Access=public)
        function nStates = getNumStatesRepresented(obj)
            nStates = length(obj.stateCPT.getRepresentedIDs()) + 1; % DP version
        end
        
        function printInfo(obj)
            printInfo@HMM(obj);
            fprintf('sHDP.alpha: %2.3f, sHDP.beta: %2.3f, eHDP.alpha: %2.3f, eHDP.beta: %2.3f\n', ...
                obj.stateCPT.alpha, obj.stateCPT.beta, obj.emissionCPT.alpha, obj.emissionCPT.beta);
        end
    end
    
    methods(Access=public) %protected
        
        %% Non-static helper functions
        function decrementTransition(obj, previousState, currentState, nextState)
            decrementTransition@HMM(obj, previousState, currentState, nextState);
            if currentState > 0
                % Then decrement the oracle in our HDP too
                obj.lastStateOracleIdx = obj.stateCPT.decrementOracle(currentState, obj.incrStepSize);
                % Now clear out a state if these decrements made currentState unrepresented
                obj.stateCPT.clearIfUnrepresented(currentState); % necessary??
            end
        end
        
        function decrementEmission(obj, currentState, currentEmission)
            decrementEmission@HMM(obj, currentState, currentEmission);
            if currentEmission > 0
                % Then decrement the oracle in our HDP too
                obj.lastEmissionOracleIdx = obj.emissionCPT.decrementOracle(currentEmission, obj.incrStepSize);
            end
        end
        
        % like regular incrementTransition, except that we must first calculate
        % the probability that this transition was drawn from the
        % oracle, and re-increment as necessary
        function incrementTransition(obj, previousState, currentState, nextState)
            incrementedCurrentState = false;
            if previousState > 0 && currentState > 0
                incrementedCurrentState = obj.stateCPT.incrementOracleWeighted(previousState, currentState, obj.incrStepSize);
            end
            if ~incrementedCurrentState && obj.lastStateOracleIdx > 0
                obj.stateCPT.incrementOracle(obj.lastStateOracleIdx, obj.incrStepSize);
            end
            incrementTransition@HMM(obj, previousState, currentState, nextState);
        end
        
        % like regular incrementEmission, except that we must first calculate
        % the probability that this emission was drawn from the
        % oracle, and re-increment as necessary
        function incrementEmission(obj, currentState, currentEmission)
            incrementedCurrentToken = false;
            if currentState > 0 && currentEmission > 0
                incrementedCurrentToken = obj.emissionCPT.incrementOracleWeighted(currentState, currentEmission, obj.incrStepSize);
            end
            if ~incrementedCurrentToken && obj.lastEmissionOracleIdx > 0
                obj.emissionCPT.incrementOracle(obj.lastEmissionOracleIdx, obj.incrStepSize);
            end
            incrementEmission@HMM(obj, currentState, currentEmission);
        end
        
        % Removes unreprested entries in our HDP, and updates the
        % hidden state and emission sequences accordingly
        function desparsifyCPT(obj)
            freeIndices = obj.stateCPT.getFreeIDs();
            for i=length(freeIndices):-1:1        % Make sure we delete from the back onwards, otherwise indexing is more complex.
                obj.S(obj.S > freeIndices(i)) = obj.S(obj.S > freeIndices(i)) - 1; % bump state IDs greater than highest zero-index down
                obj.stateCPT.countMtr(freeIndices(i),:) = [];  % delete the zero-index row in the transition counts matrix
                obj.stateCPT.countMtr(:,freeIndices(i)) = [];  % delete the zero-index column in the transition counts matrix
                obj.emissionCPT.countMtr(freeIndices(i),:) = [];  % delete the zero-index row in the emission counts matrix
                obj.stateCPT.Oracle(freeIndices(i)) = [];
            end
        end
        
        function resampleHypers(obj)
            %obj.desparsifyCPT();
            %obj.stateCPT.resampleAlpha(10, 4, 2);
            %obj.stateCPT.resampleBeta(10, 4, 2);
            %obj.emissionCPT.resampleAlpha(10, 4, 2);
            %obj.emissionCPT.resampleBeta(10, 4, 2);
            HDP_HMM.setHDPFromSequence(obj.stateCPT, obj.emissionCPT, obj.S, obj.E, obj.incrStepSize)
        end
    end
    
    methods(Access=public, Static)
        % Generate CPT from a sequence of states and emissions
        function setHDPFromSequence(stateHDP, emissionHDP, S, E, incrStepSize)
            stateHDP.reset();
            emissionHDP.reset();
            assert(length(S)==length(E));
            emissionHDP.incrementCountMatrix(S(1),E(1), incrStepSize);
            emissionHDP.incrementOracleWeighted(S(1), E(1), incrStepSize);
            stateHDP.incrementCountMatrix(1,S(1), incrStepSize);
            for t=2:length(S)
                % Update Emission HDP
                emissionHDP.incrementCountMatrix(S(t),E(t), incrStepSize);
                emissionHDP.incrementOracleWeighted(S(t), E(t), incrStepSize);
                
                % Update Transition HDP
                stateHDP.incrementCountMatrix(S(t-1),S(t), incrStepSize);
                stateHDP.incrementOracleWeighted(S(t-1),S(t), incrStepSize);
            end
            nstates = length(stateHDP.getRepresentedIDs());
            stateHDP.expandToFit(nstates, nstates);
        end
    end
end