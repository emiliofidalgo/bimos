#ifndef _RANSAC_H_
#define _RANSAC_H_

#include <set>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <limits>

#include "bimos/motionest/ParameterEstimator.h"

namespace bimos
{

/**
 * This class implements the RAndom SAmple Consensus (RANSAC) framework,
 * a framework for robust parameter estimation.
 * Given data containing outliers we estimate the model parameters using sub-sets of
 * the original data:
 * 1. Choose the minimal subset from the data for computing the exact model parameters.
 * 2. See how much of the input data agrees with the computed parameters.
 * 3. Goto step 1. This can be done up to (N choose m) times, where m is the number of
 *    data objects required for an exact estimate and N is the total number of data objects.
 * 4. Take the largest subset of objects which agreed on the parameters and compute a
 *    least squares fit using them.
 *
 * This is based on:
 * Fischler M.A., Bolles R.C.,
 * ``Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography'',
 * Communications of the ACM, Vol. 24(6), 1981.
 *
 * Hartely R., Zisserman A., "Multiple View Geometry in Computer Vision", 2001.
 *
 * The class template parameters are T - objects used for the parameter estimation
 *                                      (e.g. Point2D in line estimation,
 *                                            std::pair<Point2D,Point2D> in homography estimation).
 *                                   S - type of parameter (e.g. double).
 *
 * @author: Ziv Yaniv (zivy@isis.georgetown.edu)
 *
 */
template<class T, class S>
class RANSAC {

public:
    /**
     * Estimate the model parameters using the RANSAC framework.
     * @param parameters A vector which will contain the estimated parameters.
     *                   If there is an error in the input then this vector will be empty.
     *                   Errors are: 1. Less data objects than required for an exact fit.
     *                               2. The given data is in a singular configuration (e.g. trying to fit a circle
     *                                  to a set of colinear points).
   *                               3. The given parameter desiredProbabilityForNoOutliers is not in (0,1)
     * @param paramEstimator An object which can estimate the desired parameters using either an exact fit or a
     *                       least squares fit.
     * @param data The input from which the parameters will be estimated.
     * @param desiredProbabilityForNoOutliers The probability that at least one of the selected subsets doesn't contain an
     *                                        outlier, must be in (0,1).
     * @param outliers The list of outliers (as indices to 'data').
     * @return Returns the percentage of data used in the least squares estimate.
     */
    static double compute(std::vector<S> &parameters,
                          ParameterEstimator<T,S> *paramEstimator ,
                          std::vector<T> &data,
                          double desiredProbabilityForNoOutliers,
                          bool MSAC, // NEW AOR
                          std::vector<int> &outliers, // NEW AOR
                          double* reperror = 0); // NEW EGF


    /**
     * Estimate the model parameters using the maximal consensus set by going over ALL possible
     * subsets (brute force approach).
     * Given: n -  data.size()
     *        k - numForEstimate
     * We go over all n choose k subsets       n!
     *                                     ------------
     *                                      (n-k)! * k!
     * @param parameters A vector which will contain the estimated parameters.
     *                   If there is an error in the input then this vector will be empty.
     *                   Errors are: 1. Less data objects than required for an exact fit.
     *                               2. The given data is in a singular configuration (e.g. trying to fit a circle
     *                                  to a set of colinear points).
     * @param paramEstimator An object which can estimate the desired parameters using either an exact fit or a
     *                       least squares fit.
     * @param data The input from which the parameters will be estimated.
     * @param numForEstimate The number of data objects required for an exact fit.
     * @param outliers The list of outliers (as indices to 'data').
     * @return Returns the percentage of data used in the least squares estimate.
   *
     * NOTE: This method should be used only when n choose k is small (i.e. k or (n-k) are approximatly equal to n)
     *
     */
    static double compute(std::vector<S> &parameters,
                          ParameterEstimator<T,S> *paramEstimator ,
                          std::vector<T> &data,
                          std::vector<int> &outliers); // NEW AOR
private:

    /**
      * Compute n choose m  [ n!/(m!*(n-m)!)].
    * If choose(n,m)>std::numeric_limits<unsigned int>::max(), or there is an
    * overflow during the computations then we return
    * std::numeric_limits<unsigned int>::max(), otherwise the correct value
    * is returned.
        */
    static unsigned int choose(unsigned int n, unsigned int m);

    static void computeAllChoices(ParameterEstimator<T,S> *paramEstimator, std::vector<T> &data,
                                  bool *bestVotes, bool *curVotes, int &numVotesForBest, int startIndex, int k, int arrIndex, int *arr);

    static void estimate(ParameterEstimator<T,S> *paramEstimator, std::vector<T> &data,
                         bool *bestVotes, bool *curVotes, int &numVotesForBest, int *arr);

    class SubSetIndexComparator {
    private:
        int length;
    public:
        SubSetIndexComparator(int arrayLength) : length(arrayLength){}
        bool operator()(const int *arr1, const int *arr2) const {
            for(int i=0; i<this->length; i++) {
                if(arr1[i] < arr2[i])
                    return true;
                else if(arr1[i] > arr2[i])
                    return false;
            }
            return false;
        }
    };
};

// -------- CLASS IMPLEMENTATION -------
template<class T, class S>
double RANSAC<T,S>::compute(std::vector<S> &parameters,
                            ParameterEstimator<T,S> *paramEstimator,
                            std::vector<T> &data,
                            double desiredProbabilityForNoOutliers,
                            bool MSAC, // NEW AOR
                            std::vector<int> &outliers,
                            double* reperror) // NEW AOR
{
    int numDataObjects = data.size();
    unsigned int numForEstimate = paramEstimator->numForEstimate();

    //there are less data objects than the minimum required for an exact fit, or
    //desiredProbabilityForNoOutliers is not in (0.0,1.0)
    if(numDataObjects < numForEstimate ||
            desiredProbabilityForNoOutliers>=1.0 || desiredProbabilityForNoOutliers<=0.0)
        return 0;

    std::vector<T *> exactEstimateData;
    std::vector<T *> leastSquaresEstimateData;
    std::vector<S> exactEstimateParameters;
    int i, j, k, l, numVotesForBest, numVotesForCur, maxIndex;
    double votesForBest, votesForCur; // NEW AOR
    unsigned numTries;
    //true if data[i] agrees with the best model, otherwise false
    bool *bestVotes = new bool[numDataObjects];
    //true if data[i] agrees with the current model, otherwise false
    bool *curVotes = new bool[numDataObjects];
    //true if data[i] is NOT chosen for computing the exact fit, otherwise false
    bool *notChosen = new bool[numDataObjects];
    SubSetIndexComparator subSetIndexComparator(numForEstimate);
    std::set<int *, SubSetIndexComparator > chosenSubSets(subSetIndexComparator);
    int *curSubSetIndexes;
    double numerator = log(1.0-desiredProbabilityForNoOutliers);
    double denominator;
    //allTries is either the correct value or if there is an overflow
    //during the computation it is set to the maximal value
    //for unsigned int
    unsigned int allTries = choose(numDataObjects,numForEstimate);

    parameters.clear();
    /*srand((unsigned)time(NULL));*/ //seed random number generator // NEW AOR

    numVotesForBest = 0; //initalize with 0 so that the first computation which gives any type of fit will be set to best
    numTries = allTries; //initialize with the number of all possible subsets

    votesForBest = 0.0; // NEW AOR

    for(i=0; i<numTries; i++) {
        //randomly select data for exact model fit ('numForEstimate' objects).
        std::fill(notChosen,notChosen+numDataObjects, true);
        curSubSetIndexes = new int[numForEstimate];

        exactEstimateData.clear();

        maxIndex = numDataObjects-1;
        for(l=0; l<numForEstimate; l++) {
            //selectedIndex is in [0,maxIndex]
            int selectedIndex = (int)(((float)rand()/(float)RAND_MAX)*maxIndex + 0.5);
            for(j=-1,k=0; k<numDataObjects && j<selectedIndex; k++) {
                if(notChosen[k])
                    j++;
            }
            k--;
            exactEstimateData.push_back(&(data[k]));
            notChosen[k] = false;
            maxIndex--;
        }
        //get the indexes of the chosen objects so we can check that this sub-set hasn't been
        //chosen already
        for(l=0, j=0; j<numDataObjects; j++) {
            if(!notChosen[j]) {
                curSubSetIndexes[l] = j+1;
                l++;
            }
        }

        //check that the sub-set just chosen is unique
        std::pair< typename std::set<int *, SubSetIndexComparator >::iterator, bool> res = chosenSubSets.insert(curSubSetIndexes);

        if(res.second == true) { //first time we chose this sub set

            //use the selected data for an exact model parameter fit
            paramEstimator->estimate(exactEstimateData,exactEstimateParameters);
            //selected data is a singular configuration (e.g. three colinear points for
            //a circle fit)
            if(exactEstimateParameters.size() == 0)
                continue;
            //see how many agree on this estimate
            numVotesForCur = 0;
            votesForCur = 0.0; // NEW AOR
            std::fill(curVotes,curVotes+numDataObjects, false);
            //continue checking data until there is no chance of getting a larger consensus set
            //or all the data has been checked
            for(j=0; j<numDataObjects && numVotesForBest-numVotesForCur<numDataObjects-j+1; j++) {
                double dist; // NEW AOR
                /*if(paramEstimator->agree(exactEstimateParameters, data[j], &dist)) {*/ // NEW AOR
                if(paramEstimator->agree(exactEstimateParameters, data[j], &dist)) { // NEW AOR
                    curVotes[j] = true;
                    numVotesForCur++;
                    votesForCur += dist; // NEW AOR
                }
            }                            //found a larger consensus set?
            /*if(numVotesForCur > numVotesForBest) {*/ // NEW AOR
            bool update = (!MSAC && (numVotesForCur > numVotesForBest)) || (MSAC && (votesForCur > votesForBest)); // NEW AOR
            if (update) { // NEW AOR
                votesForBest = votesForCur; // NEW AOR
                numVotesForBest = numVotesForCur;
                std::copy(curVotes, curVotes+numDataObjects, bestVotes);
                //all data objects are inliers, terminate the search
                if(numVotesForBest == numDataObjects)
                    i=numTries;
                else {  //update the estimate of outliers and the number of iterations we need
                    denominator = log(1.0- pow((double)numVotesForCur/(double)numDataObjects, (double)(numForEstimate)));
                    numTries = (int)(numerator/denominator + 0.5);
                    //there are cases when the probablistic number of tries is greater than all possible sub-sets
                    numTries = numTries<allTries ? numTries : allTries;
                }
            }
        }
        else {  //this sub set already appeared, release memory
            delete [] curSubSetIndexes;
        }
    }

    //release the memory
    typename std::set<int *, SubSetIndexComparator >::iterator it = chosenSubSets.begin();
    typename std::set<int *, SubSetIndexComparator >::iterator chosenSubSetsEnd = chosenSubSets.end();
    while(it!=chosenSubSetsEnd) {
        delete [] (*it);
        it++;
    }
    chosenSubSets.clear();

    //compute the least squares estimate using the largest sub set
    if(numVotesForBest > 0) {
        for(j=0; j<numDataObjects; j++) {
            if(bestVotes[j])
                leastSquaresEstimateData.push_back(&(data[j]));
            else
                outliers.push_back(j); // NEW AOR
        }
        paramEstimator->leastSquaresEstimate(leastSquaresEstimateData, parameters);
    }
    delete [] bestVotes;
    delete [] curVotes;
    delete [] notChosen;

    // NEW EGF
    if (reperror)
    {
        //(*reperror) = votesForBest;
        double total_error = 0.0;
        if(numVotesForBest > 0 && parameters.size() > 0)
        {
            for(j = 0; j < (int)leastSquaresEstimateData.size(); j++)
            {
                double dist;
                paramEstimator->agree(parameters, *(leastSquaresEstimateData[j]), &dist);
                total_error += dist;
            }
            (*reperror) = total_error;
        }
        else
        {
            (*reperror) = std::numeric_limits<double>::infinity();
        }
    }

    return (double)numVotesForBest/(double)numDataObjects;
}

/*****************************************************************************/

template<class T, class S>
double RANSAC<T,S>::compute(std::vector<S> &parameters,
                            ParameterEstimator<T,S> *paramEstimator,
                            std::vector<T> &data,
                            std::vector<int> &outliers) // NEW AOR
{
    unsigned int numForEstimate = paramEstimator->numForEstimate();
    std::vector<T *> leastSquaresEstimateData;
    int numDataObjects = data.size();
    int numVotesForBest = 0;
    int *arr = new int[numForEstimate];
    short *curVotes = new short[numDataObjects];  //one if data[i] agrees with the current model, otherwise zero
    short *bestVotes = new short[numDataObjects];  //one if data[i] agrees with the best model, otherwise zero

    parameters.clear();

    //there are less data objects than the minimum required for an exact fit
    if(numDataObjects < numForEstimate)
        return 0;

    computeAllChoices(paramEstimator,data,
                      bestVotes, curVotes, numVotesForBest, 0, numForEstimate, 0, arr);

    //compute the least squares estimate using the largest sub set
    if(numVotesForBest > 0) {
        for(int j=0; j<numDataObjects; j++) {
            if(bestVotes[j])
                leastSquaresEstimateData.push_back(&(data[j]));
            else
                outliers.push_back(j); // NEW AOR
        }
        paramEstimator->leastSquaresEstimate(leastSquaresEstimateData,parameters);
    }

    delete [] arr;
    delete [] bestVotes;
    delete [] curVotes;

    return (double)numVotesForBest/(double)numDataObjects;
}

/*****************************************************************************/

template<class T, class S>
void RANSAC<T,S>::computeAllChoices(ParameterEstimator<T,S> *paramEstimator, std::vector<T> &data,
                                    bool *bestVotes, bool *curVotes, int &numVotesForBest, int startIndex, int k, int arrIndex, int *arr)
{
    //we have a new choice of indexes
    if(k==0) {
        estimate(paramEstimator, data, bestVotes, curVotes, numVotesForBest, arr);
        return;
    }
    //continue to recursivly generate the choice of indexes
    int endIndex = data.size()-k;
    for(int i=startIndex; i<=endIndex; i++) {
        arr[arrIndex] = i;
        computeAllChoices(paramEstimator, data, bestVotes, curVotes, numVotesForBest,
                          i+1, k-1, arrIndex+1, arr);
    }

}
/*****************************************************************************/

template<class T, class S>
void RANSAC<T,S>::estimate(ParameterEstimator<T,S> *paramEstimator, std::vector<T> &data,
                           bool *bestVotes, bool *curVotes, int &numVotesForBest, int *arr)
{
    std::vector<T *> exactEstimateData;
    std::vector<S> exactEstimateParameters;
    unsigned int numDataObjects;
    unsigned int numVotesForCur;
    unsigned int j;

    numDataObjects = data.size();
    std::fill(curVotes,curVotes+numDataObjects, false);
    numVotesForCur=0;

    unsigned int numForEstimate = paramEstimator->numForEstimate();

    for(j=0; j<numForEstimate; j++)
        exactEstimateData.push_back(&(data[arr[j]]));
    paramEstimator->estimate(exactEstimateData,exactEstimateParameters);
    //singular data configuration
    if(exactEstimateParameters.size()==0)
        return;

    for(j=0; j<numDataObjects; j++) {
        if(paramEstimator->agree(exactEstimateParameters, data[j])) {
            curVotes[j] = true;
            numVotesForCur++;
        }
    }
    if(numVotesForCur > numVotesForBest) {
        numVotesForBest = numVotesForCur;
        std::copy(curVotes, curVotes+numDataObjects, bestVotes);
    }
}

/*****************************************************************************/

template<class T, class S>
unsigned int RANSAC<T,S>::choose(unsigned int n, unsigned int m)
{
    double denominatorEnd, numeratorStart, numerator,denominator, i, result;
    //perform smallest number of multiplications
    if((n-m) > m) {
        numeratorStart = n-m+1;
        denominatorEnd = m;
    }
    else {
        numeratorStart = m+1;
        denominatorEnd = n-m;
    }

    for(i=numeratorStart, numerator=1; i<=n; i++)
        numerator*=i;
    for(i=1, denominator=1; i<=denominatorEnd; i++)
        denominator*=i;
    result = numerator/denominator;

    //check for overflow both in computation and in result
    if(denominator>std::numeric_limits<double>::max() ||
            numerator>std::numeric_limits<double>::max() ||
            static_cast<double>(std::numeric_limits<unsigned int>::max())<result )
        return std::numeric_limits<unsigned int>::max();
    else
        return static_cast<unsigned int>(result);
}

}

#endif //_RANSAC_H_
