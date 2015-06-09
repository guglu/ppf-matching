#include <matrix.h>
#include <mex.h>   
#include <math.h>
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
    mxArray *a_in_m, *b_in_m, *c_out_m;
    const mwSize *dims;
    double *a, *b, *c;
 //   int dimx, dimy, numdims;
 //   int i,j;
    

//associate inputs
    a_in_m = mxDuplicateArray(prhs[0]);
    b_in_m = mxDuplicateArray(prhs[1]);

//figure out dimensions
  //  dims = mxGetDimensions(prhs[0]);
  //  numdims = mxGetNumberOfDimensions(prhs[0]);
  //  dimy = (int)dims[0]; dimx = (int)dims[1];

//associate outputs
    c_out_m = plhs[0] = mxCreateDoubleMatrix(1,4,mxREAL);


//associate pointers
    a = mxGetPr(a_in_m);
    b = mxGetPr(b_in_m);
    c = mxGetPr(c_out_m);
 

    double d[3]= {b[0]-a[0], b[1]-a[1], b[2]-a[2]};
    double norm=sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
    
    c[3]=norm;
    
    d[0]/=norm;
    d[1]/=norm;
    d[2]/=norm;
    
    double cross1[3]= { a[4]*d[2]-a[5]*d[1]  ,  a[5]*d[0]-a[3]*d[2]  ,  a[3]*d[1]-a[4]*d[0] };
    double cross2[3]= { b[4]*d[2]-b[5]*d[1]  ,  b[5]*d[0]-b[3]*d[2]  ,  b[3]*d[1]-b[4]*d[0] };
    double cross3[3]= { a[4]*b[5]-a[5]*b[4]  ,  a[5]*b[3]-a[3]*b[5]  ,  a[3]*b[4]-a[4]*b[3] };
    
    c[0]=atan2( sqrt(cross1[0]*cross1[0] + cross1[1]*cross1[1] + cross1[2]*cross1[2]) ,  d[0]*a[3]+d[1]*a[4]+d[2]*a[5]  );
    c[1]=atan2( sqrt(cross2[0]*cross2[0] + cross2[1]*cross2[1] + cross2[2]*cross2[2]) ,  d[0]*b[3]+d[1]*b[4]+d[2]*b[5]  );
    c[2]=atan2( sqrt(cross3[0]*cross3[0] + cross3[1]*cross3[1] + cross3[2]*cross3[2]) ,  a[3]*b[3]+a[4]*b[4]+a[5]*b[5]  );
    return;
}

static inline void TCross(const double a[], const double b[], double c[])
{
  c[0] = (a[1])*(b[2])-(a[2])*(b[1]);
  c[1] = (a[2])*(b[0])-(a[0])*(b[2]);
  c[2] = (a[0])*(b[1])-(a[1])*(b[0]);
}