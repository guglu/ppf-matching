#include <matrix.h>
#include <mex.h>   
#include <cmath>
/* Definitions to keep compatibility with earlier versions of ML */
#ifndef MWSIZE_MAX
typedef int mwSize;
typedef int mwIndex;
typedef int mwSignedIndex;

#if (defined(_LP64) || defined(_WIN64)) && !defined(MX_COMPAT_32)
/* Currently 2^48 based on hardware limitations */
# define MWSIZE_MAX    281474976710655UL
# define MWINDEX_MAX   281474976710655UL
# define MWSINDEX_MAX  281474976710655L
# define MWSINDEX_MIN -281474976710655L
#else
# define MWSIZE_MAX    2147483647UL
# define MWINDEX_MAX   2147483647UL
# define MWSINDEX_MAX  2147483647L
# define MWSINDEX_MIN -2147483647L
#endif
#define MWSIZE_MIN    0UL
#define MWINDEX_MIN   0UL
#endif

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

//declare variables
    mxArray *a_in_m, *b_in_m, *c_out_m, *ang1_in_m, *ang2_in_m, *om1_in_m, *om2_in_m, *dist_in_m, *angLim_in_m;
    const mwSize *dims;
    double *t1, *t2, *ang1, *ang2, *om1, *om2, *dist ,*angLim, *res;


//associate inputs
    a_in_m = mxDuplicateArray(prhs[0]);
    b_in_m = mxDuplicateArray(prhs[1]);
    ang1_in_m = mxDuplicateArray(prhs[2]);
    ang2_in_m = mxDuplicateArray(prhs[3]);
    om1_in_m = mxDuplicateArray(prhs[4]);
    om2_in_m = mxDuplicateArray(prhs[5]);
    dist_in_m = mxDuplicateArray(prhs[6]);
    angLim_in_m = mxDuplicateArray(prhs[7]);

//associate outputs
    c_out_m = plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);

    
//associate pointers
    t1 = mxGetPr(a_in_m);
    t2 = mxGetPr(b_in_m);
    ang1 = mxGetPr(ang1_in_m);
    ang2 = mxGetPr(ang2_in_m);
    om1 = mxGetPr(om1_in_m);
    om2 = mxGetPr(om2_in_m);
    dist = mxGetPr(dist_in_m);
    angLim = mxGetPr(angLim_in_m);
    
    res = mxGetPr(c_out_m);
    
    
    double d[3]= {t1[0]-t2[0], t1[1]-t2[1], t1[2]-t2[2]};
    double normd=sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
    
    double phi = fabs( *ang1 - *ang2 );
    
   // double omega[3] = {om1[0]-om2[0], om1[1]-om2[1], om1[2]-om2[2]};
   // double omegaN = sqrt(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]);
    
    //if (normd < (*dist*0.10) && phi<*angLim && omegaN<*angLim)
    if (normd < (*dist*0.10) && phi<*angLim)
    {
        res[0]=1;
    }
    else
    {
        res[0]=0;
    }
     

    return;
}
