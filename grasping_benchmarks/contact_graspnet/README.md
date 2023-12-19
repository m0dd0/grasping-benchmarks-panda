# Contact Graspnet
## Use of the original Repository
The original repository of the contact graspnet has a low level of documentation and overall code quality. 
Especially the imports are not working properly when the code is used as a library.
However, refactoring the code to a library was attempted but is a lot of work.
Therefore we use some workaruonds to use the original repository directly as a library and avoid mofiying/forking it.

## Handling of the Parameter Files on GDrive
The parameter files are stored on GDrive and need to be downloaded and copied to the correct location in order to use the contact graspnet.
The problem is that doing this from a Dockerfile results in rate limiting by Google.
Therefore the parameter files are downloaded and copied next to the Dockerfile from where they are copied into the Docker image.
Howeverm this means that the Dockerfile is not self contained and the parameter files need to be downloaded manually.
Other soutions should therfore be considered for the future.

## Format of the Input Data
