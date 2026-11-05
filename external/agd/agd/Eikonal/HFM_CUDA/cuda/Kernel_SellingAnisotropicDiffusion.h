#pragma once
// Copyright 2022 Jean-Marie Mirebeau, University Paris-Sud, CNRS, University Paris-Saclay
// Distributed WITHOUT ANY WARRANTY. Licensed under the Apache License, Version 2.0, see http://www.apache.org/licenses/LICENSE-2.0

/**
This file implements a non-negative anisotropic diffusion scheme, gpu accelerated, based on :
Fehrenbach, Mirebeau, Sparse non-negative stencils for anisotropic diffusion, JMIV, 2013
*/

/** The following need to be defined in including file (example)
typedef int Int;
typedef float Scalar;
#define ndim_macro 3
*/


#if (ndim_macro == 2)
#include "Geometry2.h"
#elif (ndim_macro == 3)
#include "Geometry3.h"
#endif 

#undef bilevel_grid_macro
#include "Grid.h"

#if ced_macro
#include "CoherenceEnhancingDiffusion.h"
#endif

__constant__ Int shape_tot[ndim];
__constant__ Int size_tot; // product of shape_tot
__constant__ Scalar dx[ndim],dt; // grid scale, time step

extern "C" {
__global__ void 
anisotropic_diffusion_scheme(const Scalar * __restrict__ D_t, 
	Scalar * __restrict__ wdiag_t, Scalar * __restrict__ wneigh_t, Int * __restrict__ ineigh_t
	#if retD_macro
	,Scalar * __restrict__ retD_t
	#endif
	){

	const Int n_t = blockIdx.x*blockDim.x + threadIdx.x;
	if(n_t>=size_tot) {return;}

	// Get the position where the work is to be done.
	Int x_t[ndim];
	Grid::Position(n_t,shape_tot,x_t);

	// Optional : structure tensor transformation into diffusion tensor
	Scalar D[symdim];
	for(int i=0; i<symdim; ++i) D[i]=D_t[n_t*symdim+i];
	#if ced_macro
	Scalar lambda[ndim], mu[ndim];
	eigvalsh(D,lambda);
	ced(lambda,mu);
	map_eigvalsh(D,lambda,mu);
	#endif
	#if retD_macro
	for(int i=0; i<symdim; ++i) retD_t[n_t*symdim+i]=D[i];
	#endif
	for(int i=0,k=0;i<ndim;++i) for(int j=0;j<=i;++j,++k) D[k]/=dx[i]*dx[j];

	// Selling decomposition
	Scalar weights[decompdim]; 
	Int offsets[decompdim][ndim];
	decomp_m(D,weights,offsets);

	// Conversion to linear indices, and storage
	Scalar wsum=0;
	for(int i=0,k=0; i<decompdim; ++i){
		const Scalar weight = weights[i]/2;
		wneigh_t[n_t*decompdim+i] = weight;

		Int y_t[ndim];
		for(int s=0; s<=1; ++s,++k){
			if(s) add_vv(x_t,offsets[i],y_t); 
			else sub_vv(x_t,offsets[i],y_t);

			if(Grid::InRange_per(y_t,shape_tot)) {
				const Int ny_t = Grid::Index_per(y_t,shape_tot);
				ineigh_t[n_t*(decompdim*2)+k]   = ny_t;
				atomicAdd(wdiag_t+ny_t,weight);
				wsum+=weight;
			} // if in range
		} // for s
	} // for i
	atomicAdd(wdiag_t+n_t,wsum);
}

__global__ void 
anisotropic_diffusion_step(const Scalar * __restrict__ uold_t, Scalar * __restrict__ unew_t,
	const Scalar * __restrict__ wdiag_t, 
	const Scalar * __restrict__ wneigh_t, const Int * __restrict__ ineigh_t){

	const Int n_t = blockIdx.x*blockDim.x + threadIdx.x;
	if(n_t>=size_tot) {return;}

	const Scalar uold = uold_t[n_t];
	Scalar uinc = uold*(1-dt*wdiag_t[n_t]);
	for(int i=0,k=0; i<decompdim; ++i){
		const Scalar weight = dt*wneigh_t[n_t*decompdim+i];
		for(int j=0; j<2; ++j,++k){
			const Int ineigh = ineigh_t[n_t*(decompdim*2)+k];
			if(ineigh>=0) {
				uinc+=weight*uold_t[ineigh];
				atomicAdd(unew_t+ineigh,weight*uold);
			}
		}
	}
	atomicAdd(unew_t+n_t,uinc);
}
} // extern "C"