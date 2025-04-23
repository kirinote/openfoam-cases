#!/bin/bash

cd backGround

nprocs=18
foamDictionary system/decomposeParDict -entry numberOfSubdomains -set $nprocs

decomposePar
mpirun -np $nprocs renumberMesh -overwrite -parallel | tee log.renumberMesh
mpirun -np $nprocs overPimpleDyMFoam -parallel | tee log.solver
reconstructPar

rm -rf processor*

