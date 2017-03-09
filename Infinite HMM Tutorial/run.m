% Example run configurations for the HMM models in this demo.  This code
% was left fairly alpha (see README.txt), but the examples below work
% consistently.
% Note that the 3rd one, which starts underpowered, seems to converge to a
% locally optimal representation with fewer than 10 states.  I played with
% this a lot here and in Jurgen's code, and it just seems like a property
% of the problem itself.**  
%
% Jonathan Scholz
% jkscholz@gatech.edu
% 11/2/2011

% Build a problem
p=HMMProblem;

% Run fixed-size HMM on a 10 state representation
p.runFixed(10, 100, true, false);

% Run iHMM on 30 state representation
%p.runHDP(30, 100, true, false);

% Run iHMM on 5 state representation (interestingly, doesn't work as well)
%p.runHDP(5, 100, true, false);

% ** 
% Even Jurgen's beam sampler got stuck in this mode, which I guess means
% these modes are very pronounced and tough to walk out of.  It makes
% sense if you watch the transition and emission matrices: suboptimal
% modes end up assigning all 30 of the tokens for several of the true
% states to one state, which means that the bin counts tend to occur in
% multiples of 30 in the modes.  This aliasing of the true states would
% require resampling all 30 incorrect states at once to a new correct one,
% which is clearly unlikely.  Watching it online, you can see the counts
% drift down from 60 (or 30*x), and just when you get excited the
% super-state will gobble them back up again.  