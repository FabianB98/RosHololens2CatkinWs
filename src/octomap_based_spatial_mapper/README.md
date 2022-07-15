## OctoMap based spatial mapper

A spatial mapper which creates a voxel representation of the environment based on the approach described in [1].
Can distinguish between static (i.e. not moving) and dynamic (i.e. moving) voxels. Dynamic voxels are clustered into separate
objects and each cluster is tracked individually over time.

[1]: Asma Azim and Olivier Aycard. Detection, classification and tracking of moving objects in a 3d environment. In 2012 IEEE Intelligent Vehicles Symposium, pages 802-807. IEEE, 2012.
