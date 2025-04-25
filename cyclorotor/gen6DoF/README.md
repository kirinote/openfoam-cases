# gen6DoF.c
To compile the program:
``` 
wclean
wmake
```
Then go to `$FOAM_USER_APPBIN` where the executable has been created and run it:  
```
cd $FOAM_USER_APPBIN
./gen6DoF
```
This will generate a file "6DoF*.dat" that has to be copied into the directory of your case (at a location specified in dynamicMeshDict).
