#include "mex.h"
#include <iostream>
#include <math.h>
#include "matrix.h"

/****************************/
/* The computational routine */
void calc_Dense(double *x, double *y, double *dim, int sd)
{

int grid=5; // Size of the mask/kernel
int kin=0;
double mr = 0;

double Gs=0, Gr=0;
double S=0, Y=0, WGain=0;
/*******************/ 
for (int k=0; k<sd; k=k+1){
   if  (x[k+2*sd] > mr) { mr = x[k+2*sd]; }
}
//mexPrintf("Max Lidar range = %f\n",mr);
/*******************/ 
for (int v=0; v<dim[1]; v=v+1)
{
 for (int u=0; u<dim[2]; u=u+1)
 {
    S=0; Y=0;
     for (int k=kin; k<sd; k=k+1)
     {
        
            if(x[k+sd] <= v+dim[0]-grid) { kin=k; }
            if(x[k+sd] >= v+dim[0]+grid) { break;  }        
        
        if ( x[k] > u-grid && x[k]< u+grid && x[k+sd] > v+dim[0]-grid && x[k+sd] < v+dim[0]+grid )
        {
            Gr = x[k+2*sd]/mr;
            //Gs =  sqrt( (u - x[k])*(u - x[k]) + (v+dim[0]-x[k+sd])*(v+dim[0]-x[k+sd]) );
            Gs =  ( (u - x[k])*(u - x[k]) + (v+dim[0]-x[k+sd])*(v+dim[0]-x[k+sd]) );
            WGain = 1/sqrt(Gs*Gr);
            //mexPrintf("Filter Gain = %f\n",WGain);
            S = S + WGain;
            Y = Y + WGain*(x[k+2*sd]);
        }
     
     }
    if (S==0) {S=1;}
    y[u*(int)dim[1]  + v] = Y/S;
    
 }
}


} /*End "calc_Dense" Function*/
/****************************/

void mexFunction(
        int          nlhs,
        mxArray      *plhs[],
        int          nrhs,
        const mxArray *prhs[]
        )
{
    
    double  *Lidar;
    double  *par;
    const mwSize *dims;
    double *Y;
    
    /* Check for proper number of arguments */
        if (nrhs != 2) {
        mexErrMsgIdAndTxt("MATLAB:mexcpp:nargin",
                "MEXCPP requires 2 input arguments.");
    } else if (nlhs >= 2) {
        mexErrMsgIdAndTxt("MATLAB:mexcpp:nargout",
                "MEXCPP requires 1 output argument.");
    } 
    
    Lidar	= mxGetPr(prhs[0]);
    par     = mxGetPr(prhs[1]);
    dims    = mxGetDimensions(prhs[0]);
            
    plhs[0]= mxCreateDoubleMatrix(par[1],par[2],mxREAL);
    Y =   mxGetPr(plhs[0]);

    calc_Dense(Lidar,Y,par,(int)dims[0]);    

}








