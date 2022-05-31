## Naive spatial mapper

A naive spatial mapper which adds point clouds obtained over multiple frames into a single joint point cloud.
Does not have any special logic for removing points belonging to moving objects, so the resulting point cloud will contain ghost points of any object which (was) moved during the mapping process.
