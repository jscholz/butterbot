=================
iHMM Demo Library
=================

This directory contains code for training fixed and variable-sized HMM 
models. The implementation and example are based on the infinite Hidden
Markov Model, presented at NIPS in 2003 by Beal et al.  

This is NOT intended for use as an actual sampler for the iHMM - it's 
alpha and not remotely game-ready.  It was written as an exercise and to 
share it with my lab group at Georgia Tech.  After that meeting I had 
little excuse to work out the kinks.  One notable omission is the 
resampling of alpha and beta (the HDP concentration params) in the iHMM 
example, which I have stubs for but never finished.  

* Also, in a moment of insanity I refactored the code into a fully OOP 
implementation, which seems inadvisable in Matlab.  Factoring this way 
made the code more readable for me, but it seemed to run far slower than 
the imperative version (I blame the GC).  

** If you're looking for a usable, turn-key implementation, I suggest you 
check out Jurgen Van Gael's site: http://mlg.eng.cam.ac.uk/jurgen/

Jonathan Scholz
jkscholz@gatech.edu
11/2/2011
 