# Contact Graspnet
## Setup
### Using Dockercontainer

## Development Notes
### Use of the original Repository
The original repository of the contact graspnet has a low level of documentation and seems to be not designed to be used as a library.
Especially the imports are not working properly when the code is used as a library.
Refactoring the code to a library was attempted but is a lot of work.
Therefore we use some workaruonds to use the original repository directly as a library and avoid mofiying/forking it.

### Handling of the Parameter Files on GDrive
The parameter files are stored on GDrive and need to be downloaded and copied to the correct location in order to use the contact graspnet.
The problem is that doing this from a Dockerfile results in rate limiting by Google.
Therefore the parameter files are downloaded and copied next to the Dockerfile from where they are copied into the Docker image.
Howeverm this means that the Dockerfile is not self contained and the parameter files need to be downloaded manually.
Other soutions should therfore be considered for the future.

### Format of the Input Data
As stated above refactoring the code to a library was attempted but is a lot of work.
Therefore the wrapper simply uses the same steps as provided in the `inferece.py` file of the original repository.
However, the ´inference.py´ file expects the input data to be provided as a `.npy` file whose values are then magicallly converted to a `segmap, rgb, depth, cam_K, pc_full, pc_colors` by the `load avaiblable input data` function.
The format of these values is not documented. 
Theredore we inder the format of theses values from running the `load avaiblable input data`.
The format of the input data is as follows:
- `segmap`: numpy array of shape `(720, 1280)` with values of type `np.float`. Only integer values from `0` to `11` are used.
- `rgb`: numpy array of shape `(720, 1280, 3)` with values of type `np.uint8`. Values are in the range from `0` to `255`.
- `depth`: numpy array of shape `(720, 1280)` with values of type `np.float32`. Values are in the range from `0.0` to `6.291`.
- `cam_K`: numpy array of shape `(3, 3)` with values of type `np.float64`. Values are in the range from `0.0` to `912.72`.
- `pc_full`:
- `pc_colors`:

Knowing the format of the input data we can use a custom converter which converts the ROS message to the correct format and for all downstream steps we can use the original code of the repository.