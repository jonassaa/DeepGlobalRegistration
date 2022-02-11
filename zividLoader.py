import numpy as np
import open3d as o3d
import netCDF4


def loadPointCloudFromZivid(pcd=r"DeepGlobalRegistration/zividPointClouds/1.zdf"):
    zividPointCloud = netCDF4.Dataset(pcd,'a', format = "NETCDF4")
    return np.reshape(np.asarray(zividPointCloud["data/pointcloud"]),(1920*1200,3))

def loadRGBAFromZivid(pcd=r"DeepGlobalRegistration/zividPointClouds/1.zdf"):
    zividPointCloud = netCDF4.Dataset(pcd,'a', format = "NETCDF4")
    return np.reshape(np.asarray(zividPointCloud["data/rgba_image"]),(1920*1200,4))

def zividToO3dPointCloud(pcd=r"DeepGlobalRegistration/zividPointClouds/1.zdf",subsampleFraction = 1):
    xyz = loadPointCloudFromZivid(pcd)
    xyz = subsample(xyz,subsampleFraction)
    o3dPcd = o3d.geometry.PointCloud()
    o3dPcd.points = o3d.utility.Vector3dVector(xyz)
    return o3dPcd

def subsample(pcd,fraction = 0.5):
    step = int(1/fraction)
    return pcd[::step]

        

if __name__ == "__main__":
    loadPointCloudFromZivid()
    loadRGBAFromZivid()
    zividToO3dPointCloud()
    subsample(loadPointCloudFromZivid())