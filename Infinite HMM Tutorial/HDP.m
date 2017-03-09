classdef HDP < ConditionalProbabilityTable
    % A class for representing conditional probabilities using the
    % hierarchical dirichlet process.
    %
    % HDP presents the same interface as ConditionalProbabilityTable, but
    % provides the additional possiblity of sampling a new state.
    %
    % Jonathan Scholz
    % jkscholz@gatech.edu
    % 11/5/2011
    
    properties(Access=public) % protected
        Oracle = [];
        alpha = 1;  % Concentration parameter for Oracle
        beta = 0.5;   % Concentration parameter for low-level DP (countMtr rows)
    end
    
    methods(Access=public)
        function obj = HDP(varargin)
            if nargin >= 1
                obj.alpha = varargin{1};
            end
            if nargin >= 2
                obj.beta = varargin{2};
            end
        end
        
        function reset(obj)            
            reset@ConditionalProbabilityTable(obj);
            obj.Oracle = [];
        end
        
        function setOracle(obj, oracleVector)
            xdim = size(oracleVector,1);
            assert(xdim==1,'Oracle must be a row vector');
            obj.Oracle = oracleVector;
        end
        
        function oracleVector = getOracle(obj)
            oracleVector = obj.Oracle;
        end
        
        % Add qty probability mass to the jth component of the ith row
        function success = incrementOracle(obj, i, qty)
            success = false;
            if i > 0
                obj.expandToFit(1,i);
                obj.Oracle(i) = obj.Oracle(i) + qty;
                success = true;
            end
        end
        
        % Increments the oracle according to the probability that it
        % was drawn from to produce element j, given the current
        % counts in row i
        function success = incrementOracleWeighted(obj, i, j, qty)
            success = false;
            [w1,w2,w3] = obj.computeLevelWeightsSingle(i);
            level = HMM.sampleFromDistribution([w1,w2,w3]);
            if level == 2 || level == 3 % then we drew j from the oracle
                success = obj.incrementOracle(j, qty);
            end
        end
        
        % Add qty probability mass to the jth component of the ith row
        function success = incrementCountMatrix(obj, i, j, qty)
            obj.expandToFit(i,j);
            success = incrementCountMatrix@ConditionalProbabilityTable(obj,i,j,qty);
        end
        
        % Remove qty probability mass from the ith entry of the oracle
        % @return The index of the entry that was decremented, or zero on
        % failure (this makes it easy to re-increment the old oracle with
        % the appropriate value)
        function decremented = decrementOracle(obj, i, qty)
            decremented = 0;
            if i ~= -1 && obj.Oracle(i) >= qty
                obj.Oracle(i) = obj.Oracle(i) - qty;
                decremented = i; % In this case we return the actual index
            end
        end
        
        %% Main HDP math
        
        % Computes the probability of drawing from each level of the
        % HDP for just the specified low-level DP (count matrix row)
        %
        % @return w1 The probability of drawing from the LLDP
        % @return w2 The probability of drawing from the HLDP
        % @return w3 The probability of drawing a new component
        function [w1,w2,w3,countMtrRowsum,oracleSum] = computeLevelWeightsSingle(obj, i)
            obj.expandToFit(i,1);   % ensure we can index this row
            
            oracleSum = sum(obj.Oracle);
            countMtrRowsum = sum(obj.countMtr(i,:)); % row sums from countMtr
            
            % Compute weights of drawing from each level of the DP
            w1 = countMtrRowsum / (countMtrRowsum + obj.beta);     % prob of drawing from low-level DP
            w2 = obj.beta / (countMtrRowsum + obj.beta) * oracleSum/(oracleSum + obj.alpha); % prob of drawing from high-level DP
            w3 = obj.beta / (countMtrRowsum + obj.beta) * obj.alpha/(oracleSum + obj.alpha); % prob of generating a new state
        end
        
        % Computes the probability of drawing from each level of the
        % HDP for each entry in the low-level DP matrix (count matrix).
        %
        % @return w1 The probability of drawing from the LLDP
        % @return w2 The probability of drawing from the HLDP
        % @return w3 The probability of drawing a new component
        function [w1,w2,w3,countMtrRowsums,oracleSum] = computeLevelWeightsAll(obj)
            oracleSum = sum(obj.Oracle);
            countMtrRowsums = sum(obj.countMtr, 2); % row sums from countMtr
            
            % Compute weights of drawing from each level of the DP
            pll = countMtrRowsums./(countMtrRowsums + obj.beta);     % prob of not defaulting out of low-level DP
            phl = oracleSum/(oracleSum + obj.alpha);                 % prob of not defaulting out of high-level DP
            w1 = pll;
            w2 = (1-pll) * phl; % prob of drawing from high-level DP
            w3 = (1-pll) * (1-phl); % prob of generating a new state
            %assert(1-pll == obj.beta./(countMtrRowsums + obj.beta));
            %assert(HMM.equals(sum(w1+w2+w3),size(obj.countMtr,1)));
        end
        
        % Computes an n+1 by m+1 matrix of conditional probabilities
        % given the provided countMtr & Oracle counters, where n is the
        % number of rows of the countMtr matrix, and m is the number of
        % columns (Oracle is nx1).
        function CPT = getCPT(obj)
            assert(size(obj.countMtr,2) == length(obj.Oracle));
            
            [w1,w2,w3,countMtrRowsums,oracleSum] = obj.computeLevelWeightsAll();
            if oracleSum == 0
                oracleProbs = zeros(1,length(obj.Oracle));
                %oracleProbs = ones(1,length(obj.Oracle))/length(obj.Oracle); % if oracleSum is zero then w2 will be zero anyway...
            else
                oracleProbs = obj.Oracle/oracleSum; % normalized state counts vector
            end
            
            % Compute the CPT over represented components
            lldp_normed = bsxfun(@rdivide, obj.countMtr, countMtrRowsums);
            lldp_normed(isnan(lldp_normed)) = 0; % NaN protector
            a = bsxfun(@times, lldp_normed, w1);  % weighted countMtr transition probs
            b = bsxfun(@times, repmat(oracleProbs, size(obj.countMtr,1), 1), w2);
            CPT = [a + b, w3]; % append column with probabilities of generating a new element
            CPT = [CPT;(oracleSum/(oracleSum + obj.alpha)) .* oracleProbs, obj.alpha/(oracleSum + obj.alpha)]; % append row with probabilities given unrepresented element
            
            % For debugging:
            perform_checks(CPT);
            function perform_checks(CPT)
                try
                    assert(HMM.equals(sum(sum(CPT,2)),size(CPT,1)));
                    assert(max(max(CPT)) <= 1);
                    assert(min(min(CPT)) >= 0);
                catch err
                    disp(err);
                    keyboard
                end
            end
        end
        
        % Resample alpha (using Escobar & West 1995) (oracle concentration param)
        % ** samples a new alpha param for the oracle which is gamma distributed
        % with parameter proportional to the number of states that the oracle
        % currently represents
        function resampleAlpha(obj, numi, priorGammaA, priorGammaB)
            %priorGammaA = 4;
            %priorGammaB = 2;
            
            k = length(obj.Oracle) + 1;
            m = sum(obj.Oracle);
            for iter = 1:numi
                mu = betarnd(obj.alpha + 1, m);
                pi_mu = 1 / (1 + (m * (priorGammaB - log(mu))) / (priorGammaA + k - 1)  );
                if rand() < pi_mu
                    obj.alpha = gamrnd(priorGammaA + k, 1.0 / (priorGammaB - log(mu)));
                else
                    obj.alpha = gamrnd(priorGammaA + k - 1, 1.0 / (priorGammaB - log(mu)));
                end
            end
        end
        
        % Idea: beta should reflect the probability of defaulting out of the lldp.
        %  Thus, for the given number of counts we have for each state,  beta
        %  should be consistent with the actual number of times we popped out
        %  according to our matrix M above.  We compute this by simulation: first
        %  we compute the probability of popping out of each row of the lldp.  Then
        %  we flip a coin weighted by this probability for each state, indicating
        %  an overall number of times we drew from the lldp.
        % ** Note that the EV of a gamma RV is a*b, which means that we should get an
        % answer in the neighborhood of (sum(sum(M)) - sum(s)) / (-sum(log(w))).
        % ** Since a gamma(n,b) distribution can be regarded as the distribution of a
        % sum of n exp(b) RV's, it makes sense to think of this as a sum of ... hmm
        % as what again?  There's something special about sum(sum(M)) - sum(s),
        % which looks like the number of oracle draws minus the number of simulated
        % lldp draws on one pass through the state space.
        function resampleBeta(obj, numi, priorGammaA, priorGammaB)
            for iter = 1:numi
                w = betarnd(obj.beta + 1, sum(obj.countMtr,2)); % beta RV propto rowsums in the low-level count matrix (ie roughly the probs of defaulting out of each row of lldp)
                rowSums = sum(obj.countMtr,2);
                p =  obj.beta ./ (rowSums + obj.beta);
                %p = sum(obj.countMtr,2)/obj.beta; % rowsums scaled by old lldp alpha param (propto prob of drawing from lldp)
                %p = p ./ (p+1); % scale (is he using 1 as the "self-trans alpha" param from Beal 2002?)
                % ** [p is same thing as sum(N,2) ./ (sum(N,2) + ialpha); i.e., it's exactly the probability of drawing from the lldp for each state given current transition counts]
                s = binornd(1, p); % binomial RV given p (i.e. flip coin for whether each state drew from lldp)
                obj.beta = gamrnd(priorGammaA + sum(obj.Oracle) - sum(s), 1.0 / (priorGammaB - sum(log(w)))); % draw new alpha ~ gamma, with param
                if isnan(obj.beta)
                    keyboard
                end
            end
        end
        
        %% HDP-specific helper functions
        function bool = isRepresented(obj, idx)
            if idx > length(obj.Oracle) || (obj.Oracle(idx)==0 && sum(obj.countMtr(:,idx))==0)
                bool = false;
            else
                bool = true;
            end
        end
        
        function representedIDs = getRepresentedIDs(obj)
            % Returns a list of indices for all represented IDs
            rep_LL = find(sum(obj.countMtr,1)); % nonzero column sums
            rep_HL = find(obj.Oracle);          % nonzero oracle entries
            representedIDs = union(rep_LL, rep_HL);
        end
        
        function freeIDs = getFreeIDs(obj)
            % Returns a list of indices for all represented IDs
            rep_LL = find(sum(obj.countMtr,1) == 0); % zero column sums
            rep_HL = find(obj.Oracle == 0);          % zero oracle entries
            freeIDs = intersect(rep_LL, rep_HL);
        end
        
        % A function to clear out the counts if a specific became
        % unrepresented.  Should be called after the decrement operation,
        % before getCPT
        function clearIfUnrepresented(obj, i)
            if ~obj.isRepresented(i) %obj.Oracle(i)==0 && sum(obj.countMatrix(:,i))==0
                obj.countMtr(i,:) = 0;
            end
        end
    end
    
    methods(Access=public) % protected
        function expandToFit(obj, nRowsNeeded, nColsNeeded)
            nRows = size(obj.countMtr,1);
            nCols = size(obj.countMtr,2);
            
            if nRowsNeeded > nRows
                obj.countMtr = [obj.countMtr; zeros(nRowsNeeded - nRows, nCols)];
            end
            
            if nColsNeeded > nCols
                assert(length(obj.Oracle) == nCols);
                obj.Oracle = [obj.Oracle, zeros(1,nColsNeeded - nCols)];
                obj.countMtr = [obj.countMtr, zeros(max(nRowsNeeded,nRows), nColsNeeded - nCols)];
            end
        end
    end
end

