### hmmlearn is a set of algorithms for UNSUPERVISED LEARNING and INFERANCE of HMM 
# https://youtu.be/mnGN9BUs0HI?si=364HyXlEVnTeuZfj

import numpy as np
from hmmlearn.hmm import MultinomialHMM # from normal distribution

### HMM implementation

# Initial probabilities of 2 states
startprob = np.array([0.5, 0.5]) 

# Trasnition matrix
transmat = np.array([[0.7, 0.3], 
                    [0.3, 0.7]])  # somma righe = 1

# Covariance matrix
covar = np.array([[0.9, 0.1], 
                  [0.2, 0.8]]) # somma righe = 1

# Create the model
model = MultinomialHMM(n_components=2, 
                       startprob_prior= startprob, 
                       transmat_prior=transmat)

# Create observations
X = [[0, 0, 1, 0], 
     [0, 0, 0, 0], 
     [1, 1, 1, 0], 
     [0, 0, 1, 1]]

# Fit data on model to find the transition matrix
# Try to learn the transition matrix from observation
model.fit(X)
print(model.transmat_)

# Prediction 
prob = model.decode(np.array([0, 1, 0, 1]).reshape(4,1))
print(np.exp(prob[0]))

# Genarate new samples
Xnew = model.sample(100)
print(Xnew)
