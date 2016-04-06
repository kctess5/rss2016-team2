from random import uniform
from bisect import bisect

MIN_WEIGHT = .1

# Returns index of a random element, likelyhood proportional to value
# assumes weights are greater than or equal to zero
def nondeterministic_weighted_index(weights):
    cdf = [weights[0]+MIN_WEIGHT]
    for i in xrange(1,len(weights)):
        cdf.append(weights[i] + cdf[i-1] + MIN_WEIGHT)
    X = uniform(0.,cdf[-1])
    return bisect(cdf, X)

