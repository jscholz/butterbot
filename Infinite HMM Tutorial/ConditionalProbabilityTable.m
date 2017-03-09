classdef ConditionalProbabilityTable < handle
    % An interface and basic implementation for defining conditional
    % probability tables.
    %
    % The intended application is for use in an HMM, which learns this
    % distrubution using Gibbs sampling.  For these reason, the public
    % interface consists of functions for moving probability mass around
    % (incrementing and decrement), and getting the CPT (presumably to
    % evaluate various likelihoods).
    %
    % Note that the internal representation is just an un-normalized matrix
    % of counters, which is more convenient to work with in our sampler.
    %
    % Jonathan Scholz
    % jkscholz@gatech.edu
    % 11/2/2011
    
    properties(Access=public) % protected
        % The primary data data structure for the CPT - an nXm matrix of
        % counters
        countMtr = [];
    end
    
    methods(Access=public)
        function reset(obj)
            obj.countMtr = [];
        end

        % Add qty probability mass to the jth component of the ith row
        function success = incrementCountMatrix(obj, i, j, qty)
            obj.countMtr(i, j) = obj.countMtr(i, j) + qty;
            success = true;
        end
        
        % Remove qty probability mass from the jth component of the ith row
        function success = decrementCountMatrix(obj, i, j, qty)
            success = false;
            assert(i <= size(obj.countMtr,1) && j <= size(obj.countMtr,2));
            if obj.countMtr(i, j) >= qty
                obj.countMtr(i, j) = obj.countMtr(i, j) - qty;
                success = true;
            end
        end
        
        % Retreive the full count matrix
        function countMtr = getCountMatrix(obj)
            countMtr = obj.countMtr;
        end
        
        % Compute the CPT
        function CPT = getCPT(obj)
            CM = obj.countMtr + 0.03; % pad count matrix to smooth out likelihoods a bit
            rowsums = sum(CM,2);
            CPT = bsxfun(@rdivide, CM, rowsums);
        end
        
        % Set the internal Count Matrix
        function setCountMatrix(obj, countMtr)
            obj.countMtr = countMtr;
        end
    end
end