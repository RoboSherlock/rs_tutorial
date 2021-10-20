#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <robosherlock/types/all_types.h>
//RS
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include <rs_tutorial/types/all_types.h>
#include <pcl/common/centroid.h>


using namespace uima;


class MyFirstAnnotator : public Annotator
{
private:
  float test_param;

public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("test_param", test_param);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    outInfo("Test param =  " << test_param);
    cas.get(VIEW_CLOUD,*cloud_ptr);

    outInfo("Cloud size: " << cloud_ptr->points.size());
    outInfo("took: " << clock.getTime() << " ms.");

    rs::Scene scene = cas.getScene();
    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);
    int idx = 0;
    for (auto cluster:clusters)
    {
      rs::ObjectHypothesis &c = cluster;
      pcl::PointIndices indices;
      rs::conversion::from(((rs::ReferenceClusterPoints)c.points()).indices(),indices);
      outInfo("ObjectHypothesis has "<<indices.indices.size()<<" points");

      Eigen::Vector4d pCentroid;
      pcl::compute3DCentroid(*cloud_ptr,indices, pCentroid);
      rs_tutorial::MyFirstAnnotation annotation  = rs::create<rs_tutorial::MyFirstAnnotation>(tcas);
      rs_tutorial::Centroid centroid = rs::create<rs_tutorial::Centroid>(tcas);
      centroid.x.set(pCentroid[0]);
      centroid.y.set(pCentroid[1]);
      centroid.z.set(pCentroid[2]);
      annotation.centroid.set(centroid);
      annotation.clusterId.set(idx++);
      c.annotations.append(annotation);
    }
    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(MyFirstAnnotator)
