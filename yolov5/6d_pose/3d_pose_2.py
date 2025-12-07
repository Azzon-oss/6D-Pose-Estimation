"""
This code estimates 3D pose given model and sample point clouds as input using Open3D library.
It follows a global registration followed by refine registration (ICP) approach.

"""

from __future__ import division, print_function, unicode_literals  # To support both python 2 and python 3
import numpy as np
import math
import open3d as o3d

#首先，代码使用给定的体素大小（“voxel_size”）对原始样本点云和模型点云进行下采样操作。
# 下采样后的结果分别存储在"sample_down"和"model_down"变量中。
#接下来，代码使用下采样后的样本点云和模型点云，以及给定的搜索半径（“radius_normal”）对估计法线进行操作。
# 估计法线的方法是使用"KDTreeSearchParamHybrid"参数，在指定的搜索半径和最大最近邻点数量下进行估计。估计得到的法线存储在对应的点云中。
#然后，代码使用另一个搜索半径（“radius_feature”）对下采样后的样本点云和模型点云进行特征计算.
# 具体地，这段代码计算了FPFH特征（Fast Point Feature Histograms），使用的搜索参数与估计法线相似，但搜索半径和最大最近邻点数量略有不同。
#最后，代码返回了原始样本点云、原始模型点云、下采样后的样本点云、下采样后的模型点云以及计算得到的样本FPFH特征和模型FPFH特征
class pose3D(object):
    def __init__(self, model_orig_pcd, sample_pcd):
        """
        Arguments:
            model_orig_pcd: model point cloud
            sample_pcd: sample point cloud
        """
        self.model_pcd_original = model_orig_pcd
        self.XYZ_model = np.asarray(self.model_pcd_original.points)
        self.XYZ_min_model = np.min(self.XYZ_model, axis=0)
        self.XYZ_max_model = np.max(self.XYZ_model, axis=0)
        self.diameter_model = np.sqrt(np.square(self.XYZ_max_model[0] - self.XYZ_min_model[0]) +
                                      np.square(self.XYZ_max_model[1] - self.XYZ_min_model[1]) +
                                      np.square(self.XYZ_max_model[2] - self.XYZ_min_model[2]))
        #计算模型的直径大小，总的点云数量
        print("初始状态下的场景点云")
        o3d.visualization.draw_geometries([sample_pcd])
        print("初始状态下的模型点云")
        o3d.visualization.draw_geometries([model_orig_pcd])
        print("初始状态下的模型和场景点云")
        o3d.visualization.draw_geometries([model_orig_pcd + sample_pcd])
        self.model_pcd = o3d.geometry.PointCloud()  # To keep resized model points that can match the size of a sample
        # point cloud.
        print("初始化，计算模型的直径大小，总的点云数量")
        self.sample_pcd = sample_pcd



#名为"prepare_dataset"，该方法接受三个参数：model，sample和voxel_size。让我为您解释代码的功能：

    # 首先，代码会打印出下采样的体素大小，即输入的voxel_size。然后，使用o3d.geometry.PointCloud.voxel_down_sample方法对sample和model进行下采样操作，
    # 将下采样后的结果保存到sample_down和model_down变量中。
    # 接下来，代码计算估计法线的搜索半径。将voxel_size乘以1赋值给radius_normal变量。然后，
    # 代码使用o3d.geometry.PointCloud.estimate_normals方法对sample_down和model_down进行估计法线操作，
    # 使用o3d.geometry.KDTreeSearchParamHybrid来设置搜索参数，包括搜索半径和最大最近邻点数量。同样，代码还对原始的sample和model进行了估计法线操作。
    # 然后，代码计算FPFH特征的搜索半径。将voxel_size乘以2赋值给radius_feature变量。之后，
    # 代码使用o3d.pipelines.registration.compute_fpfh_feature方法分别对sample_down和model_down计算FPFH特征。
    # 在特征计算中，同样使用了o3d.geometry.KDTreeSearchParamHybrid来设置搜索参数，包括搜索半径和最大最近邻点数量。
    def prepare_dataset(self, model, sample, voxel_size):

        print(":: Downsample with a voxel size %.3f." % voxel_size)
        sample_down = o3d.geometry.PointCloud.voxel_down_sample(sample, voxel_size)
        model_down = o3d.geometry.PointCloud.voxel_down_sample(model, voxel_size)
        #对模型和场景进行下采样
        radius_normal = voxel_size * 1  # 2 (default) 1 (new camera) 1.5 (old camera)
        print(":: Estimate normal with search radius %.3f." % radius_normal)
        o3d.geometry.PointCloud.estimate_normals(sample_down, o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius_normal, max_nn=10))  # 30 (default) 10 (new camera) 20 (old  camera)
        o3d.geometry.PointCloud.estimate_normals(model_down, o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius_normal, max_nn=10))
        o3d.geometry.PointCloud.estimate_normals(sample, o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius_normal, max_nn=10))
        o3d.geometry.PointCloud.estimate_normals(model, o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius_normal, max_nn=10))
        #对场景，，模型进行估计，以及下采样后的法线估计
        #sample_down是输入的降采样后的点云数据，radius_normal是设定的搜索半径
        radius_feature = voxel_size * 2  # 5 (default) 2 (new camera)  3(old camera)
        print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
        source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(sample_down,
                o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=40))  # 100 (default) 40 (new camera)
        # 60 (old camera)

        target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(model_down,
                o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=40))
        return sample, model, sample_down, model_down, source_fpfh, target_fpfh
        # 计算二者下采样后的FPFH特征

#首先，代码根据voxel_size计算了一个距离阈值，存储在distance_threshold变量中。
#然后，代码打印了一些关于执行RANSAC（随机采样一致性）全局配准的信息，包括采样点云和模型点云的体素大小以及选择的距离阈值。
#接下来，代码调用了o3d.pipelines.registration.registration_ransac_based_on_feature_matching方法进行基于特征匹配的RANSAC配准。
# 该方法的参数包括：sample_down和model_down（下采样后的采样点云和模型点云），
# sample_fpfh和model_fpfh（采样点云和模型点云的FPFH特征），0.075（初始对应点对的最大距离），
# o3d.pipelines.registration.TransformationEstimationPointToPoint(False)（位姿估计方法，使用点对点的估计方法），
# 4（RANSAC采样一致性算法的迭代次数），以及两个 o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength
# 和 o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance（用于过滤不良对应点对），
# 最后一个参数是 o3d.pipelines.registration.RANSACConvergenceCriteria（用于指定RANSAC算法的收敛条件）。
#最后，代码返回了配准的结果result。

    def execute_global_registration(self, sample_down, model_down, sample_fpfh, model_fpfh, voxel_size):
        distance_threshold = voxel_size * 1.0  # 1.5 (default) 1.0 (old camera)
        print(":: 随机采样RANSAC registration on downsampled point clouds.")
        print("   下采样体素大小Since the downsampling voxel size is %.3f," % voxel_size)
        print("   距离阈值 we use a liberal distance threshold %.3f." % distance_threshold)
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            sample_down, model_down, sample_fpfh, model_fpfh, 0.075,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 4,
            [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.075)],
            o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
        print("全局注册global_registration: ")
        print(result)
        return result

#首先，代码根据voxel_size计算了一个距离阈值，存储在distance_threshold变量中。
#然后，代码打印了关于使用给定的距离阈值执行快速全局配准的信息。
#接下来，代码调用了o3d.pipelines.registration.registration_fgr_based_on_feature_matching方法进行基于特征匹配的快速全局配准。
# 该方法的参数包括：sample_down和model_down（下采样后的采样点云和模型点云），sample_fpfh和model_fpfh（采样点云和模型点云的FPFH特征），
# 以及 o3d.pipelines.registration.FastGlobalRegistrationOption(maximum_correspondence_distance=distance_threshold)
# （用于指定最大对应点距离的快速全局配准选项）
    def execute_fast_global_registration(self, sample_down, model_down, sample_fpfh, model_fpfh, voxel_size):
        distance_threshold = voxel_size * 0.2  # 0.5 (default) 0.2 (new camera)
        print(":: 快速全局注册阈值Apply fast global registration with distance threshold %.3f" % distance_threshold)
        result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            sample_down, model_down, sample_fpfh, model_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(maximum_correspondence_distance=distance_threshold))
        print("使用快速全局注册fast_global_registration:", result)
        return result
#首先，代码根据voxel_size计算了一个较小的距离阈值，存储在distance_threshold变量中。
#然后，代码打印了一些关于应用基于点到平面的ICP（迭代最近点）算法进行配准的信息，以及本次使用的距离阈值。
#接下来，代码调用了o3d.pipelines.registration.registration_icp方法进行基于点到平面的ICP配准。
# 该方法的参数包括：sample和model（原始的采样点云和模型点云），distance_threshold（距离阈值），result_global.transformation（全局配准结果的变换矩阵），
# 以及 o3d.pipelines.registration.TransformationEstimationPointToPlane()（位姿估计方法，使用点到平面的估计方法）。
#最后，代码返回了配准的结果result。
    #点到平面的ICP（迭代最近点）算法进行配准的信息，以及本次使用的距离阈值。
    def refine_registration(self, sample, model, sample_fpfh, model_fpfh, voxel_size, result_global):
        distance_threshold = voxel_size * 0.2  # 0.4 (default) 0.2 (new camera) 0.3 (old camera)
        print(":: 使用点对平面的icp对原模型Point-to-plane ICP registration is applied on original point")
        print("   clouds to refine the alignment. This time we use a strict")
        print("   distance threshold %.3f." % distance_threshold)
        result = o3d.pipelines.registration.registration_icp(sample, model,
                distance_threshold,
                result_global.transformation,              # result_ransac.transformation or result_fast.transformation
                o3d.pipelines.registration.TransformationEstimationPointToPlane())  # TransformationEstimationPointToPlane()
        # or TransformationEstimationPointToPoint()
        print("点对平面的结果是:",result)
        return result
#首先，代码定义了一个名为voxel_size的变量，用于指定体素大小。
#然后，代码调用了self.prepare_dataset方法，该方法将sample和model准备成适合配准的格式，
# 并返回了更新后的sample、model、sample_down、model_down、sample_fpfh和model_fpfh。
#接下来，代码调用了self.execute_fast_global_registration方法，将sample_down、model_down、sample_fpfh和model_fpfh传入进行快速全局配准。该方法返回了配准结果result_global。
#接下来，代码使用result_global的变换矩阵对sample_down进行变换。
#然后，代码调用了self.refine_registration方法，将sample、model、sample_fpfh、model_fpfh、voxel_size和result_global传入进行ICP配准。
#该方法返回了配准结果result_icp。
#接下来，代码使用result_icp的变换矩阵对sample进行变换。
#然后，代码从result_icp的变换矩阵中提取出旋转矩阵。
#接下来，代码使用旋转矩阵计算出对应的四元数。
#然后，代码将model和sample分别改变成绿色和红色。
#最后，代码返回旋转矩阵和四元数。
    def combined_registration(self, sample, model):
            voxel_size = 0.006  # 0.001      #0.05 (default) # means 5cm for the dataset
            sample, model, sample_down, model_down, sample_fpfh, model_fpfh = self.prepare_dataset(model, sample,
                                                                                                   voxel_size)
            print("开始准备数据，下采样模型和场景，计算各自的fpfh特征")
            # result_global = self.execute_global_registration(sample_down, model_down, sample_fpfh, model_fpfh,
            # voxel_size) # Global using RANSAC
            print("执行快速全局匹配fast_global_registration")
            result_global = self.execute_fast_global_registration(sample_down, model_down, sample_fpfh, model_fpfh,
                                                                  voxel_size)  # Fast global

            sample_down.transform(result_global.transformation)
            # print result_global
            # print 'Global T:', result_global.transformation
            print("执行点到平面的icp")
            result_icp = self.refine_registration(sample, model, sample_fpfh, model_fpfh, voxel_size, result_global)
            sample.transform(result_icp.transformation)
            # print result_icp
            # print 'ICP T:', result_icp.transformation
            rotation_matrix = result_icp.transformation[0:3, 0:3]
            print("输出旋转矩阵")
            # print rotation_matrix
            quaternion = self.quaternion_from_matrix(rotation_matrix, isprecise=False)
            # print 'Quaternion:', quaternion
            model.paint_uniform_color([1, 0, 0])  # Model is with green color
            sample.paint_uniform_color([0, 1, 0])
            return rotation_matrix, quaternion

    def quaternion_from_matrix(self, matrix, isprecise=False):
        """
        Return quaternion from rotation matrix.

        If isprecise is True, the input matrix is assumed to be a precise rotation matrix and a faster algorithm is
        used.

        """
        m = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
        if isprecise:
            q = np.empty((4, ))
            t = np.trace(m)
            if t > m[3, 3]:
                q[0] = t
                q[3] = m[1, 0] - m[0, 1]
                q[2] = m[0, 2] - m[2, 0]
                q[1] = m[2, 1] - m[1, 2]
            else:
                i, j, k = 0, 1, 2
                if m[1, 1] > m[0, 0]:
                    i, j, k = 1, 2, 0
                if m[2, 2] > m[i, i]:
                    i, j, k = 2, 0, 1
                t = m[i, i] - (m[j, j] + m[k, k]) + m[3, 3]
                q[i] = t
                q[j] = m[i, j] + m[j, i]
                q[k] = m[k, i] + m[i, k]
                q[3] = m[k, j] - m[j, k]
                q = q[[3, 0, 1, 2]]
            q *= 0.5 / math.sqrt(t * m[3, 3])
        else:
            m00 = m[0, 0]
            m01 = m[0, 1]
            m02 = m[0, 2]
            m10 = m[1, 0]
            m11 = m[1, 1]
            m12 = m[1, 2]
            m20 = m[2, 0]
            m21 = m[2, 1]
            m22 = m[2, 2]
            # symmetric matrix k
            k = np.array([[m00 - m11 - m22, 0.0, 0.0, 0.0],
                 [m01 + m10, m11 - m00 - m22, 0.0, 0.0],
                 [m02 + m20, m12 + m21, m22 - m00 - m11, 0.0],
                 [m21 - m12, m02 - m20, m10 - m01, m00 + m11 + m22]])
            k /= 3.0
            # quaternion is eigenvector of k that corresponds to largest eigenvalue
            w, V = np.linalg.eigh(k)
            q = V[[3, 0, 1, 2], np.argmax(w)]
        if q[0] < 0.0:
            np.negative(q, q)
        return q



#首先，代码将self.sample_pcd、self.XYZ_model、self.diameter_model和self.model_pcd分别赋值给变量sample_pcd、XYZ_model、diameter_model和model_pcd。
#然后，代码将sample_pcd中的点云数据转换为一个numpy.ndarray，并计算出点云数据的最小坐标XYZ_min和最大坐标XYZ_max。
#接下来，代码计算出采样点云的直径diameter_sample，并将点云数据的形状重新调整为(-1, 3)。
#然后，代码将重新调整后的点云数据赋值给sample_pcd的points属性，并为sample_pcd设置统一的绿色。
#接下来，代码计算了一个比例ratio，用于将模型点云的大小进行调整，使其与采样点云匹配。
#然后，代码将调整后的模型点云坐标XYZ_model_new赋值给model_pcd的points属性，并为model_pcd设置统一的红色。
#接下来，代码使用o3d.visualization.draw_geometries方法显示了两个点云model_pcd和sample_pcd的叠加效果。
#然后，代码调用了self.combined_registration方法，将model_pcd和sample_pcd作为参数传入，返回了估计的旋转矩阵rotation_matrix_estimated和四元数quaternion_estimated。
#接下来，代码打印了估计的旋转矩阵rotation_matrix_estimated。
#然后，代码为model_pcd设置统一的红色，为sample_pcd设置统一的绿色。
#最后，代码使用o3d.visualization.draw_geometries方法再次显示了两个点云model_pcd和sample_pcd的叠加效果。

    def estimate_3Dpose(self):

        sample_pcd = self.sample_pcd
        XYZ_model = self.XYZ_model
        diameter_model = self.diameter_model
        model_pcd = self.model_pcd

        numpy_point_data = np.asarray(sample_pcd.points)
        print(numpy_point_data.shape)
        XYZ_min = np.min(numpy_point_data, axis=0)
        XYZ_max = np.max(numpy_point_data, axis=0)

        print("输出场景中边幅的最大和最小")
        print('minimum and maximum: ', XYZ_min, XYZ_max)
        diameter_sample = np.sqrt(np.square(XYZ_max[0] - XYZ_min[0]) + np.square(XYZ_max[1] - XYZ_min[1]) +
                                  np.square(XYZ_max[2] - XYZ_min[2]))
        numpy_point_data = np.reshape(numpy_point_data, (-1, 3))
        sample_pcd.points = o3d.utility.Vector3dVector(numpy_point_data)
        # sample_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])  # Flip it, otherwise the
        # pointcloud will be upside down
        sample_pcd.paint_uniform_color([0, 1, 0])

        ratio = diameter_sample / diameter_model  # Using diameter from pointcloud
        print("场景点云大小和模型大小的比例")
        print('Ratio = mushroom_diameter/mushroom_diameter_model: ', ratio)
        XYZ_model_new = XYZ_model
        #XYZ_model_new = XYZ_model * 0.001  # Resize the model points to match that of a sample
        print("乘以这个比例去适配场景大小")
        model_pcd.points = o3d.utility.Vector3dVector(XYZ_model_new)
        #o3d.io.write_point_cloud("model_new.ply", model_pcd)
        # model_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])  # Flip it, otherwise the
        # pointcloud will be upside down
        model_pcd.paint_uniform_color([1, 0, 0])
        print("经过比例因子调整后的大小")
        o3d.visualization.draw_geometries([model_pcd + sample_pcd])
        print("开始进行综合计算")
        rotation_matrix_estimated, quaternion_estimated = self.combined_registration(model_pcd, sample_pcd)
        print('rotation_matrix_estimated and rotation_vector_estimated: ', rotation_matrix_estimated)
        model_pcd.paint_uniform_color([1, 0, 0])
        sample_pcd.paint_uniform_color([0, 1, 0])
        print("输出配准后的两个点云文件")
        o3d.visualization.draw_geometries([model_pcd + sample_pcd])


# Main function
def main():
    model_orig_pcd = o3d.io.read_point_cloud("./pcd/model_real.pcd")
    sample_pcd = o3d.io.read_point_cloud("./scene/bottle3.ply")  # Any sample point cloud can be given here
    Pose = pose3D(model_orig_pcd, sample_pcd)
    Pose.estimate_3Dpose()


# Execute from the interpreter
if __name__ == "__main__":
    main()
