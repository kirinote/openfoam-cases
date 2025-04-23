#!/bin/bash

cd airfoil1
blockMesh
snappyHexMesh -overwrite | tee log.snappyHexMesh
extrudeMesh
createPatch -overwrite
transformPoints -translate '(0 0.07 0)'
cd ..

cd airfoil2
blockMesh
snappyHexMesh -overwrite | tee log.snappyHexMesh
extrudeMesh
createPatch -overwrite
transformPoints -translate '(0 -0.07 0)'
cd ..

cd airfoil3
blockMesh
snappyHexMesh -overwrite | tee log.snappyHexMesh
extrudeMesh
createPatch -overwrite
transformPoints -translate '(-0.07 0 0)'
cd ..

cd airfoil4
blockMesh
snappyHexMesh -overwrite | tee log.snappyHexMesh
extrudeMesh
createPatch -overwrite
transformPoints -translate '(0.07 0 0)'
cd ..

cd backGround
blockMesh
mergeMeshes . ../airfoil1 -overwrite
mergeMeshes . ../airfoil2 -overwrite
mergeMeshes . ../airfoil3 -overwrite
mergeMeshes . ../airfoil4 -overwrite
topoSet
topoSet -dict system/topoSetDict_movingZone

rm -r 0
cp -r 0_org 0

checkMesh | tee log.checkMesh
setFields | tee log.setFields

cd ..

